/**
 * @file kortex3_hardware_interface.cpp
 * @brief Implementation of the ros2_control hardware interface for the Kinova Link6 robot.
 * @author Anas Houssaini
 */

#include "kortex3_hardware/kortex3_hardware_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <thread>
#include <string>
#include <iostream>
#include <fstream>
#include <algorithm>

namespace kortex3_driver
{
const rclcpp::Logger Kortex3HardwareInterface::LOGGER =
  rclcpp::get_logger("Kortex3HardwareInterface");

// ============================================================================
// GripperController Implementation
// ============================================================================

bool GripperController::initialize(std::shared_ptr<k_api::RouterMQTT> router, const rclcpp::Logger& logger)
{
  RCLCPP_INFO(logger, "Initializing gripper '%s' with Modbus ID %u", joint_name_.c_str(), modbus_id_);

  modbus_wrapper_ = std::make_shared<slick::com::ModbusClientWrapper>(router, modbus_id_);

  if (modbus_wrapper_->TryInitConnection() != slick::com::ModbusError::Ok) {
    RCLCPP_ERROR(logger, "Failed to connect to gripper '%s' via Modbus.", joint_name_.c_str());
    return false;
  }

  RCLCPP_INFO(logger, "Gripper '%s' Modbus connection established.", joint_name_.c_str());

  gripper_ = std::make_unique<MyFingerGripper>(modbus_wrapper_);
  initialized_ = true;

  // Activate the gripper
  RCLCPP_INFO(logger, "Activating gripper '%s'...", joint_name_.c_str());

  gripper_->FreezeGripper();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  gripper_->ReadRegister();

  gripper_->SetActivateRequest();
  gripper_->SendRequest();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  gripper_->ReadRegister();

  // Poll until activation completes
  const auto activation_timeout = std::chrono::seconds(10);
  auto activation_start = std::chrono::steady_clock::now();
  bool activation_complete = false;

  while (std::chrono::steady_clock::now() - activation_start < activation_timeout)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    if (!gripper_->ReadRegister())
    {
      RCLCPP_WARN(logger, "Failed to read gripper '%s' status during activation.", joint_name_.c_str());
      continue;
    }

    if (gripper_->GetActivateEcho())
    {
      activation_complete = true;
      RCLCPP_INFO(logger, "Gripper '%s' activation completed successfully.", joint_name_.c_str());
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      break;
    }
    else
    {
      uint8_t status = gripper_->GetStatus();
      RCLCPP_DEBUG(logger, "Gripper '%s' activating... (STA=%d)", joint_name_.c_str(), status);
    }
  }

  if (!activation_complete)
  {
    RCLCPP_ERROR(logger, "Gripper '%s' activation failed or timed out.", joint_name_.c_str());
    return false;
  }

  RCLCPP_INFO(logger, "Gripper '%s' activation complete.", joint_name_.c_str());
  return true;
}

std::optional<double> GripperController::readPosition(std::mutex& mutex, const rclcpp::Logger& logger)
{
  // Rate-limit BEFORE touching the mutex
  const auto now = std::chrono::steady_clock::now();
  if (now < next_poll_) {
    return position_;  // return cached value
  }

  // Try to acquire the mutex without blocking
  std::unique_lock<std::mutex> lk(mutex, std::try_to_lock);
  if (!lk.owns_lock()) {
    // Another Modbus op in-flight; try again soon without blocking
    next_poll_ = now + std::chrono::milliseconds(10);
    return position_;
  }

  if (!initialized_) {
    return std::nullopt;
  }

  // Read once; if it fails, reconnect and retry once
  bool ok = gripper_->ReadRegister();
  if (!ok) {
    RCLCPP_WARN(logger, "ReadRegister() failed for gripper '%s'; reconnecting and retrying once", joint_name_.c_str());
    modbus_wrapper_->CloseConnection();
    if (modbus_wrapper_->TryInitConnection() == slick::com::ModbusError::Ok) {
      ok = gripper_->ReadRegister();
    }
  }

  // Set next poll time regardless
  next_poll_ = now + poll_period_;

  if (!ok) {
    return std::nullopt;
  }

  // Decode & cache. Robotiq position register: 0x00 = fully open, 0xFF = fully
  // closed. Report the joint angle in radians [0, max_angle_] to match the URDF
  // joint (0.0 = fully open, max_angle_ = fully closed) so RViz visualization is
  // correct via joint_state_broadcaster -> robot_state_publisher.
  const uint8_t raw = gripper_->GetPosition();
  position_ = (static_cast<double>(raw) / 255.0) * max_angle_;
  return position_;
}

void GripperController::sendCommand(double position_radians, std::mutex& mutex)
{
  // Gate BEFORE locking to avoid needless contention
  const auto now = std::chrono::steady_clock::now();

  // Limit command rate
  if (now < next_send_) return;

  // Command domain is joint radians [0, max_angle_]: 0.0 = fully open,
  // max_angle_ = fully closed (matches the URDF joint and MoveIt SRDF).
  // Clamp into range and skip tiny, redundant updates.
  const double clamped = std::clamp(position_radians, 0.0, max_angle_);
  if (last_cmd_pos_ == last_cmd_pos_ &&   // not NaN
      std::abs(clamped - last_cmd_pos_) < 0.0001)  // rad
  {
    return;
  }

  // Serialize Modbus access
  std::lock_guard<std::mutex> lk(mutex);
  if (!initialized_ || !gripper_) return;

  // Robotiq position register: 0x00 = fully open, 0xFF = fully closed.
  const uint8_t pos = static_cast<uint8_t>((clamped / max_angle_) * 255.0);
  const uint8_t spd = 255;

  // Best-effort write sequence
  gripper_->ClearGoToRequest();
  (void)gripper_->SendRequest();

  gripper_->SetPositionRequest(pos);
  gripper_->SetSpeedRequest(spd);
  gripper_->SetGoToRequest();
  (void)gripper_->SendRequest();

  last_cmd_pos_ = clamped;
  next_send_ = now + cmd_period_;
}

void GripperController::shutdown(std::mutex& mutex)
{
  std::lock_guard<std::mutex> lk(mutex);
  if (modbus_wrapper_) modbus_wrapper_->CloseConnection();
  gripper_.reset();
  modbus_wrapper_.reset();
  initialized_ = false;
}

// ============================================================================
// Kortex3HardwareInterface Implementation
// ============================================================================

Kortex3HardwareInterface::Kortex3HardwareInterface()
  : mqtt_port_(1883),
    udp_feedback_port_(10001),
    actuator_count_(6), // Default, updated from robot during activation.
    in_fault_(false),
    node_ptr_(nullptr),
    gripper_name_(""),
    use_internal_bus_gripper_comm_(false),
    gripper_a_(9),   // Default Modbus ID 9
    gripper_b_(10)   // Default Modbus ID 10
{
}

hardware_interface::CallbackReturn Kortex3HardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  RCLCPP_INFO(LOGGER, "Initializing Kortex3 Hardware Interface...");
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  info_ = info;
  // Parse hardware parameters from the URDF.
  robot_ip_ = info_.hardware_parameters.at("robot_ip");
  username_ = info_.hardware_parameters.at("username");
  password_ = info_.hardware_parameters.at("password");
  gripper_name_ = info_.hardware_parameters.at("gripper");

  if (info_.hardware_parameters.count("mqtt_port"))
  {
    mqtt_port_ = std::stoi(info_.hardware_parameters.at("mqtt_port"));
  }
  if (info_.hardware_parameters.count("udp_feedback_port"))
  {
    udp_feedback_port_ = std::stoi(info_.hardware_parameters.at("udp_feedback_port"));
  }

  // Configure gripper communication
  if (
    (info_.hardware_parameters["use_internal_bus_gripper_comm"] == "true") ||
    (info_.hardware_parameters["use_internal_bus_gripper_comm"] == "True"))
  {
    use_internal_bus_gripper_comm_ = true;
    RCLCPP_INFO(LOGGER, "Using internal bus communication for grippers!");
  }

  // Configure Gripper A (optional)
  if (info_.hardware_parameters.count("gripper_joint_name") &&
      !info_.hardware_parameters.at("gripper_joint_name").empty())
  {
    gripper_a_.joint_name_ = info_.hardware_parameters.at("gripper_joint_name");
    RCLCPP_INFO(LOGGER, "Gripper A joint name: '%s'", gripper_a_.joint_name_.c_str());

    // Modbus ID is optional, will use default (9) if not specified
    if (info_.hardware_parameters.count("gripper_modbus_id"))
    {
      gripper_a_.modbus_id_ = static_cast<uint16_t>(std::stoul(info_.hardware_parameters.at("gripper_modbus_id")));
      RCLCPP_INFO(LOGGER, "Gripper A Modbus ID: %u", gripper_a_.modbus_id_);
    }
    else
    {
      RCLCPP_INFO(LOGGER, "Gripper A Modbus ID not specified, using default: %u", gripper_a_.modbus_id_);
    }

    // Closed-angle limit (URDF <limit upper>) is optional; defaults to 0.8 (2f_85). Use 0.7 for 2f_140.
    if (info_.hardware_parameters.count("gripper_max_angle"))
    {
      gripper_a_.max_angle_ = std::stod(info_.hardware_parameters.at("gripper_max_angle"));
      RCLCPP_INFO(LOGGER, "Gripper A max angle: %.4f rad", gripper_a_.max_angle_);
    }
  }
  else
  {
    RCLCPP_INFO(LOGGER, "Gripper A joint name not specified, Gripper A will not be initialized.");
  }

  // Configure Gripper B (optional)
  if (info_.hardware_parameters.count("gripper_b_joint_name") &&
      !info_.hardware_parameters.at("gripper_b_joint_name").empty())
  {
    gripper_b_.joint_name_ = info_.hardware_parameters.at("gripper_b_joint_name");
    RCLCPP_INFO(LOGGER, "Gripper B joint name: '%s'", gripper_b_.joint_name_.c_str());

    // Modbus ID is optional, will use default (10) if not specified
    if (info_.hardware_parameters.count("gripper_b_modbus_id"))
    {
      gripper_b_.modbus_id_ = static_cast<uint16_t>(std::stoul(info_.hardware_parameters.at("gripper_b_modbus_id")));
      RCLCPP_INFO(LOGGER, "Gripper B Modbus ID: %u", gripper_b_.modbus_id_);
    }
    else
    {
      RCLCPP_INFO(LOGGER, "Gripper B Modbus ID not specified, using default: %u", gripper_b_.modbus_id_);
    }

    // Closed-angle limit (URDF <limit upper>) is optional; defaults to 0.8 (2f_85). Use 0.7 for 2f_140.
    if (info_.hardware_parameters.count("gripper_b_max_angle"))
    {
      gripper_b_.max_angle_ = std::stod(info_.hardware_parameters.at("gripper_b_max_angle"));
      RCLCPP_INFO(LOGGER, "Gripper B max angle: %.4f rad", gripper_b_.max_angle_);
    }
  }
  else
  {
    RCLCPP_INFO(LOGGER, "Gripper B joint name not specified, Gripper B will not be initialized.");
  }

  // Initialize state and command vectors.
  joint_velocities_cmd_.resize(actuator_count_, 0.0);
  joint_positions_.resize(actuator_count_, 0.0);
  joint_velocities_.resize(actuator_count_, 0.0);
  joint_torques_.resize(actuator_count_, 0.0);

  // Initialize gripper command positions
  gripper_a_.command_position_ = std::numeric_limits<double>::quiet_NaN();
  gripper_a_.position_ = std::numeric_limits<double>::quiet_NaN();
  gripper_b_.command_position_ = std::numeric_limits<double>::quiet_NaN();
  gripper_b_.position_ = std::numeric_limits<double>::quiet_NaN();

  // Verify that the URDF's joint count matches the expected count.
  // Count configured grippers
  int gripper_joint_count = 0;
  if (use_internal_bus_gripper_comm_ && !gripper_name_.empty())
  {
    if (!gripper_a_.joint_name_.empty()) gripper_joint_count++;
    if (!gripper_b_.joint_name_.empty()) gripper_joint_count++;
  }

  int expected_joints_number = actuator_count_ + gripper_joint_count;
  if (info_.joints.size() != expected_joints_number)
  {
    RCLCPP_ERROR(LOGGER,
      "URDF configuration error: Expected %d joints (%zu arm + %d gripper), but got %zu.",
      expected_joints_number, actuator_count_, gripper_joint_count, info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Initialize ROS 2 node and interfaces for external communication.
  node_ptr_ = std::make_shared<rclcpp::Node>("kortex3_hardware_node");
  wrench_publisher_ = node_ptr_->create_publisher<geometry_msgs::msg::WrenchStamped>(
      "kortex/tool_wrench", 10);
  set_operating_mode_service_ = node_ptr_->create_service<kortex3_hardware::srv::SetOperatingMode>(
      "kortex3_hardware/set_operating_mode",
      std::bind(&Kortex3HardwareInterface::handle_set_operating_mode,
                this, std::placeholders::_1, std::placeholders::_2));
  clear_faults_service_ = node_ptr_->create_service<kortex3_hardware::srv::ClearFaults>(
      "kortex3_hardware/clear_faults",
      std::bind(&Kortex3HardwareInterface::handle_clear_faults,
                this, std::placeholders::_1, std::placeholders::_2));
  simulate_estop_service_ = node_ptr_->create_service<kortex3_hardware::srv::SimulateEstop>(
      "kortex3_hardware/simulate_estop",
      std::bind(&Kortex3HardwareInterface::handle_simulate_estop,
                this, std::placeholders::_1, std::placeholders::_2));
  run_program_service_ = node_ptr_->create_service<kortex3_hardware::srv::RunProgram>(
      "kortex3_hardware/run_program",
      std::bind(&Kortex3HardwareInterface::handle_run_program,
                this, std::placeholders::_1, std::placeholders::_2));
  list_programs_service_ = node_ptr_->create_service<kortex3_hardware::srv::ListPrograms>(
      "kortex3_hardware/list_programs",
      std::bind(&Kortex3HardwareInterface::handle_list_programs,
                this, std::placeholders::_1, std::placeholders::_2));
  stop_program_service_ = node_ptr_->create_service<kortex3_hardware::srv::StopProgram>(
      "kortex3_hardware/stop_program",
      std::bind(&Kortex3HardwareInterface::handle_stop_program,
                this, std::placeholders::_1, std::placeholders::_2));
  get_program_status_service_ = node_ptr_->create_service<kortex3_hardware::srv::GetProgramStatus>(
      "kortex3_hardware/get_program_status",
      std::bind(&Kortex3HardwareInterface::handle_get_program_status,
                this, std::placeholders::_1, std::placeholders::_2));
  list_protection_zones_service_ = node_ptr_->create_service<kortex3_hardware::srv::ListProtectionZones>(
      "kortex3_hardware/list_protection_zones",
      std::bind(&Kortex3HardwareInterface::handle_list_protection_zones,
                this, std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(LOGGER, "Hardware Interface successfully initialized.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Kortex3HardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(LOGGER, "Configuring Kortex3 Hardware Interface...");
  // Configuration is handled in on_activate where connections are established.
  RCLCPP_INFO(LOGGER, "Configuration successful.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Kortex3HardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(LOGGER, "Activating Kortex3 Hardware Interface...");
  try
  {
    // 1. Create MQTT connection for low-frequency commands.
    router_mqtt_ = std::make_shared<k_api::RouterMQTT>(robot_ip_, mqtt_port_);
    router_mqtt_->SpinProcess(std::chrono::milliseconds{1});
    session_mqtt_ = std::make_shared<k_api::Session::SessionClient>(router_mqtt_.get());
    auto mqtt_session_info = k_api::Session::CreateSessionInfo();
    mqtt_session_info.set_username(username_);
    mqtt_session_info.set_password(password_);
    mqtt_session_info.set_session_inactivity_timeout(20000);
    mqtt_session_info.set_connection_inactivity_timeout(10000);
    session_mqtt_->CreateSession(mqtt_session_info);
    base_mqtt_ = std::make_shared<k_api::Base::BaseClient>(router_mqtt_.get());
    program_runner_ = std::make_shared<k_api::ProgramRunner::ProgramRunnerClient>(router_mqtt_.get());
    protection_zone_ = std::make_shared<k_api::ProtectionZone::ProtectionZoneClient>(router_mqtt_.get());

    // 2. Check power state and turn on the robot if necessary.
    check_and_power_on_robot();

    // 3. Create UDP connection for high-frequency feedback.
    transport_udp_feedback_ = std::make_unique<k_api::TransportClientUdp>();
    transport_udp_feedback_->connect(robot_ip_, udp_feedback_port_);
    router_udp_feedback_ = std::make_unique<k_api::RouterClient>(
      transport_udp_feedback_.get(),
      [](k_api::KError err) {
        RCLCPP_ERROR(rclcpp::get_logger("Kortex3HardwareInterface"),
                     "UDP Router error: %s", err.toString().c_str());
      });
    session_udp_ = std::make_unique<k_api::SessionManager>(router_udp_feedback_.get());
    auto udp_session_info = k_api::Session::CreateSessionInfo();
    udp_session_info.set_username(username_);
    udp_session_info.set_password(password_);
    udp_session_info.set_session_inactivity_timeout(20000);
    udp_session_info.set_connection_inactivity_timeout(10000);
    session_udp_->CreateSession(udp_session_info);
    base_cyclic_udp_ = std::make_shared<k_api::BaseCyclic::BaseCyclicClient>(
      router_udp_feedback_.get());

    // 4. Get the actual actuator count from the robot and update if different.
    auto actuator_info = base_mqtt_->GetActuatorCount();
    if (actuator_info.count() != actuator_count_)
    {
      RCLCPP_WARN(LOGGER,
        "Robot reports %d actuators, but URDF expected %zu. Using robot's count.",
        actuator_info.count(), actuator_count_);
      actuator_count_ = actuator_info.count();
    }

    // 5. Set the initial operating mode for velocity control.
    change_operating_mode(k_api::Common::OPERATING_MODE_AUTO);
    last_operating_mode_ = k_api::Common::OPERATING_MODE_AUTO;

    // 7. Initialize grippers if using internal bus communication
    if (use_internal_bus_gripper_comm_ && !gripper_name_.empty())
    {
      // Initialize Gripper A if joint name was specified
      if (!gripper_a_.joint_name_.empty())
      {
        if (!gripper_a_.initialize(router_mqtt_, LOGGER))
        {
          RCLCPP_ERROR(LOGGER, "Failed to initialize Gripper A.");
          return hardware_interface::CallbackReturn::ERROR;
        }

        // First read from Gripper A
        auto opt_gripper_a_position = gripper_a_.readPosition(gripper_mtx_, LOGGER);
        if (!opt_gripper_a_position.has_value())
        {
          RCLCPP_WARN(LOGGER, "Failed to read Gripper A position on activation.");
          return hardware_interface::CallbackReturn::ERROR;
        }
        // Seed the command with the current angle (radians) so the gripper holds
        // its position on startup instead of jerking.
        double gripper_a_initial_angle = opt_gripper_a_position.value();
        RCLCPP_INFO(LOGGER, "Gripper A initial position is '%.4f' rad (0=open, %.2f=closed).",
                    gripper_a_initial_angle, gripper_a_.max_angle_);

        gripper_a_.command_position_ = gripper_a_initial_angle;
        gripper_a_.sendCommand(gripper_a_initial_angle, gripper_mtx_);
      }

      // Initialize Gripper B if joint name was specified
      if (!gripper_b_.joint_name_.empty())
      {
        if (!gripper_b_.initialize(router_mqtt_, LOGGER))
        {
          RCLCPP_ERROR(LOGGER, "Failed to initialize Gripper B.");
          return hardware_interface::CallbackReturn::ERROR;
        }

        // First read from Gripper B
        auto opt_gripper_b_position = gripper_b_.readPosition(gripper_mtx_, LOGGER);
        if (!opt_gripper_b_position.has_value())
        {
          RCLCPP_WARN(LOGGER, "Failed to read Gripper B position on activation.");
          return hardware_interface::CallbackReturn::ERROR;
        }
        // Seed the command with the current angle (radians) so the gripper holds
        // its position on startup instead of jerking.
        double gripper_b_initial_angle = opt_gripper_b_position.value();
        RCLCPP_INFO(LOGGER, "Gripper B initial position is '%.4f' rad (0=open, %.2f=closed).",
                    gripper_b_initial_angle, gripper_b_.max_angle_);

        gripper_b_.command_position_ = gripper_b_initial_angle;
        gripper_b_.sendCommand(gripper_b_initial_angle, gripper_mtx_);
      }

      if (!gripper_a_.joint_name_.empty() && !gripper_b_.joint_name_.empty())
      {
        RCLCPP_INFO(LOGGER, "Both grippers initialized successfully.");
      }
      else if (!gripper_a_.joint_name_.empty())
      {
        RCLCPP_INFO(LOGGER, "Gripper A initialized successfully.");
      }
      else if (!gripper_b_.joint_name_.empty())
      {
        RCLCPP_INFO(LOGGER, "Gripper B initialized successfully.");
      }
    }

    RCLCPP_INFO(LOGGER, "Kortex3 Hardware Interface successfully activated.");
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  catch (const std::exception& ex)
  {
    RCLCPP_ERROR(LOGGER, "Exception during activation: %s", ex.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
}

hardware_interface::CallbackReturn Kortex3HardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(LOGGER, "Deactivating Kortex3 Hardware Interface...");
  try
  {
    // Gracefully shut down the robot and network connections in order.
    if (base_mqtt_)
    {
      change_operating_mode(k_api::Common::OPERATING_MODE_MONITORED_STOP);
    }
    if (session_udp_)
    {
      try { session_udp_->CloseSession(); }
      catch (const std::exception &e) {
        RCLCPP_ERROR(LOGGER, "Error closing UDP session: %s", e.what());
      }
    }
    if (transport_udp_feedback_)
    {
      try { transport_udp_feedback_->disconnect(); }
      catch (const std::exception &e) {
        RCLCPP_ERROR(LOGGER, "Error disconnecting UDP transport: %s", e.what());
      }
    }
    if (session_mqtt_)
    {
      try { session_mqtt_->CloseSession(); }
      catch (const std::exception &e) {
        RCLCPP_ERROR(LOGGER, "Error closing MQTT session: %s", e.what());
      }
    }
    if (router_mqtt_)
    {
      router_mqtt_->SpinProcess(std::chrono::milliseconds{0});
    }

    // Shutdown grippers that were initialized
    if (!gripper_a_.joint_name_.empty())
    {
      gripper_a_.shutdown(gripper_mtx_);
    }
    if (!gripper_b_.joint_name_.empty())
    {
      gripper_b_.shutdown(gripper_mtx_);
    }
  }
  catch (const std::exception& ex)
  {
    RCLCPP_ERROR(LOGGER, "Error during deactivation: %s", ex.what());
  }

  // Reset ROS 2 interfaces and pointers.
  set_operating_mode_service_.reset();
  clear_faults_service_.reset();
  simulate_estop_service_.reset();
  run_program_service_.reset();
  list_programs_service_.reset();
  stop_program_service_.reset();
  get_program_status_service_.reset();
  list_protection_zones_service_.reset();
  node_ptr_.reset();

  RCLCPP_INFO(LOGGER, "Kortex3 Hardware Interface deactivated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
Kortex3HardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  std::vector<string> arm_joint_names;
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    RCLCPP_DEBUG(LOGGER, "export_state_interfaces for joint: %s", info_.joints[i].name.c_str());
    if (info_.joints[i].name == gripper_a_.joint_name_)
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &gripper_a_.position_));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &gripper_a_.velocity_));
    }
    else if (info_.joints[i].name == gripper_b_.joint_name_)
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &gripper_b_.position_));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &gripper_b_.velocity_));
    }
    else
    {
      arm_joint_names.emplace_back(info_.joints[i].name);
    }
  }

  for (std::size_t i = 0; i < arm_joint_names.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      arm_joint_names[i], hardware_interface::HW_IF_POSITION, &joint_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      arm_joint_names[i], hardware_interface::HW_IF_VELOCITY, &joint_velocities_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      arm_joint_names[i], hardware_interface::HW_IF_EFFORT, &joint_torques_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
Kortex3HardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  std::vector<string> arm_joint_names;
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    if (info_.joints[i].name == gripper_a_.joint_name_)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &gripper_a_.command_position_));
    }
    else if (info_.joints[i].name == gripper_b_.joint_name_)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &gripper_b_.command_position_));
    }
    else
    {
      arm_joint_names.emplace_back(info_.joints[i].name);
    }
  }

  for (std::size_t i = 0; i < arm_joint_names.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      arm_joint_names[i], hardware_interface::HW_IF_VELOCITY, &joint_velocities_cmd_[i]));
  }
  return command_interfaces;
}

hardware_interface::return_type Kortex3HardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Read gripper states
  if (use_internal_bus_gripper_comm_ && !gripper_name_.empty())
  {
    if (!gripper_a_.joint_name_.empty())
    {
      gripper_a_.readPosition(gripper_mtx_, LOGGER);
    }
    if (!gripper_b_.joint_name_.empty())
    {
      gripper_b_.readPosition(gripper_mtx_, LOGGER);
    }
  }
  
  static int read_call_count = 0;
  read_call_count++;

  if (read_call_count % 1000 == 0) {
    RCLCPP_DEBUG(LOGGER, "read() called %d times", read_call_count);
  }

  try {
    auto feedback = base_cyclic_udp_->RefreshFeedback();

    // 1. Update joint states from high-frequency feedback.
    // Kortex API provides position/velocity in degrees, convert to radians for ROS.
    for (size_t i = 0; i < feedback.actuators_size() && i < actuator_count_; ++i) {
      const auto &act = feedback.actuators(i);
      joint_positions_[i]  = act.position() * M_PI / 180.0;
      joint_velocities_[i] = act.velocity() * M_PI / 180.0;
      joint_torques_[i]    = act.torque();
    }

    // 2. Detect arm state changes and handle faults.
    auto new_state = feedback.base().active_state();
    if (new_state != last_arm_state_) {
      RCLCPP_INFO(LOGGER, "Arm state changed: %s -> %s",
        Kinova::Api::Common::ArmState_Name(last_arm_state_).c_str(),
        Kinova::Api::Common::ArmState_Name(new_state).c_str());
      last_arm_state_ = new_state;
    }

    // If a fault occurs, set the internal flag and log an error with recovery instructions.
    if (new_state == Kinova::Api::Common::ARMSTATE_IN_FAULT ||
        new_state == Kinova::Api::Common::ARMSTATE_IN_FAULT_POWERED_OFF)
    {
      in_fault_ = true;

      // Rate limiting: only log full error message once per second
      auto now = std::chrono::steady_clock::now();
      auto time_since_last_log = std::chrono::duration_cast<std::chrono::seconds>(
          now - last_fault_log_time_);

      if (!fault_recently_logged_ || time_since_last_log.count() >= 1) {
        if (!fault_recently_logged_) {
          // First fault detection - log full error message
          RCLCPP_ERROR(LOGGER, "═══════════════════════════════════════════════════════════");
          RCLCPP_ERROR(LOGGER, "ROBOT FAULT DETECTED - State: %s",
                       Kinova::Api::Common::ArmState_Name(new_state).c_str());
          RCLCPP_ERROR(LOGGER, "RECOVERY STEPS:");
          RCLCPP_ERROR(LOGGER, "  1. Check robot teach pendant for error details");
          RCLCPP_ERROR(LOGGER, "  2. Clear faults: ros2 service call /kortex3_hardware/clear_faults \\");
          RCLCPP_ERROR(LOGGER, "       kortex3_hardware/srv/ClearFaults \"{}\"");
          RCLCPP_ERROR(LOGGER, "  3. If issue persists, power cycle the robot");
          RCLCPP_ERROR(LOGGER, "═══════════════════════════════════════════════════════════");
          fault_recently_logged_ = true;
        } else {
          // Periodic update after 1 second
          RCLCPP_WARN(LOGGER, "Robot still in fault state: %s",
                      Kinova::Api::Common::ArmState_Name(new_state).c_str());
        }
        last_fault_log_time_ = now;
      }

      rclcpp::spin_some(node_ptr_);
      return hardware_interface::return_type::OK;
    }
    // 3. Detect operating mode changes.
    auto new_mode = feedback.base().operating_mode();
    if (new_mode != last_operating_mode_) {
      RCLCPP_INFO( LOGGER, "Operating mode changed: %s -> %s",
        Kinova::Api::Common::OperatingModeType_Name(last_operating_mode_).c_str(),
        Kinova::Api::Common::OperatingModeType_Name(new_mode).c_str());
      last_operating_mode_ = new_mode;
    }
    // 3.5. Track program runner status for velocity command blocking.
    try {
      auto current_program_status = program_runner_->GetStatus();
      auto new_status = current_program_status.status();

      // Detect when program finishes (transitions from active to inactive)
      bool was_active = is_program_active(last_program_status_);
      bool is_active = is_program_active(new_status);

      if (was_active && !is_active) {
        program_end_time_ = std::chrono::steady_clock::now();
        RCLCPP_INFO(LOGGER, "Program execution ended (status: %d -> %d). Velocity commands will be blocked for 1 second.",
                    last_program_status_, new_status);
      }

      // Log status changes
      if (new_status != last_program_status_) {
        RCLCPP_DEBUG(LOGGER, "Program runner status changed: %d -> %d",
                    last_program_status_, new_status);
        last_program_status_ = new_status;
      }
    }
    catch (const k_api::KDetailedException &ex) {
      // Don't spam logs if program runner is not available or busy
      RCLCPP_DEBUG(LOGGER, "Failed to get program status in read(): %s", ex.what());
    }
    // Handle pending controller deactivation (scheduled from handle_clear_faults)
    if (pending_controller_deactivation_) {
      std::cout << "\n=== Executing Scheduled Controller Deactivation ===" << std::endl;
      RCLCPP_INFO(LOGGER, "Sending async request to deactivate %zu controller(s)...",
                  controllers_to_deactivate_.size());

      // Fire off the deactivation request in a detached thread - DON'T wait for response
      std::thread([controllers = controllers_to_deactivate_]() {
        try {
          auto temp_node = std::make_shared<rclcpp::Node>("controller_deactivate_async");
          auto temp_client = temp_node->create_client<controller_manager_msgs::srv::SwitchController>(
              "/controller_manager/switch_controller");

          if (!temp_client->wait_for_service(std::chrono::milliseconds(500))) {
            std::cout << "WARNING: Controller manager service not available" << std::endl;
            return;
          }

          auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
          request->activate_controllers = {};
          request->deactivate_controllers = controllers;
          request->strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
          request->activate_asap = false;
          request->timeout = rclcpp::Duration::from_seconds(10.0);

          // Send request asynchronously and detach - let it complete in background
          auto future = temp_client->async_send_request(request);
          std::cout << "Controller deactivation request sent asynchronously" << std::endl;

          // Spin to allow request to be sent, but don't block waiting for response
          rclcpp::executors::SingleThreadedExecutor executor;
          executor.add_node(temp_node);
          executor.spin_some();

        } catch (const std::exception& ex) {
          std::cout << "Exception in async controller deactivation: " << ex.what() << std::endl;
        }
      }).detach();

      std::cout << "Controller deactivation request fired asynchronously. Will verify in subsequent cycles." << std::endl;

      // Clear the flag - we've sent the request
      pending_controller_deactivation_ = false;
      // Set countdown to verify after ~1000 cycles (about 1 second at 1kHz)
      deactivation_verify_countdown_ = 1000;
      std::cout << "=== END Controller Deactivation Request ===" << std::endl;
    }

    // Verify controller deactivation after countdown
    if (deactivation_verify_countdown_ > 0) {
      deactivation_verify_countdown_--;
      if (deactivation_verify_countdown_ == 0 && !controllers_to_deactivate_.empty()) {
        std::cout << "\n=== Verifying Controller Deactivation ===" << std::endl;
        auto still_active = get_active_motion_controllers();

        bool all_deactivated = true;
        for (const auto& controller : controllers_to_deactivate_) {
          if (std::find(still_active.begin(), still_active.end(), controller) != still_active.end()) {
            std::cout << "  ✗ Controller '" << controller << "' is still ACTIVE" << std::endl;
            RCLCPP_WARN(LOGGER, "Controller '%s' is still active after deactivation request", controller.c_str());
            all_deactivated = false;
          } else {
            std::cout << "  ✓ Controller '" << controller << "' is INACTIVE" << std::endl;
          }
        }

        if (all_deactivated) {
          std::cout << "SUCCESS: All controllers successfully deactivated!" << std::endl;
          RCLCPP_INFO(LOGGER, "Successfully verified all motion controllers are deactivated.");

          // Switch to AUTO mode now that controllers are safely deactivated
          change_operating_mode(k_api::Common::OPERATING_MODE_AUTO);
          last_operating_mode_ = k_api::Common::OPERATING_MODE_AUTO;

          // Now reactivate the controllers asynchronously
          std::cout << "\n=== Reactivating Controllers ===" << std::endl;
          RCLCPP_INFO(LOGGER, "Reactivating %zu controller(s)...", controllers_to_deactivate_.size());

          std::thread([controllers = controllers_to_deactivate_]() {
            try {
              auto temp_node = std::make_shared<rclcpp::Node>("controller_activate_async");
              auto temp_client = temp_node->create_client<controller_manager_msgs::srv::SwitchController>(
                  "/controller_manager/switch_controller");

              if (!temp_client->wait_for_service(std::chrono::milliseconds(500))) {
                std::cout << "WARNING: Controller manager service not available for reactivation" << std::endl;
                return;
              }

              auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
              request->activate_controllers = controllers;  // Reactivate these
              request->deactivate_controllers = {};  // Not deactivating anything
              request->strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
              request->activate_asap = true;
              request->timeout = rclcpp::Duration::from_seconds(10.0);

              // Send request asynchronously
              auto future = temp_client->async_send_request(request);
              std::cout << "Controller reactivation request sent asynchronously" << std::endl;

              // Spin to allow request to be sent
              rclcpp::executors::SingleThreadedExecutor executor;
              executor.add_node(temp_node);
              executor.spin_some();

            } catch (const std::exception& ex) {
              std::cout << "Exception in async controller reactivation: " << ex.what() << std::endl;
            }
          }).detach();

          std::cout << "Controller reactivation request fired. Controllers should be active shortly." << std::endl;

        } else {
          std::cout << "WARNING: Some controllers remain active" << std::endl;
          RCLCPP_WARN(LOGGER, "Some motion controllers remain active after deactivation.");
        }

        // Clear the list after verification
        controllers_to_deactivate_.clear();
        std::cout << "=== END Verification ===" << std::endl;
      }
    }

    // 4. Publish external tool wrench telemetry.
    if (wrench_publisher_ && feedback.has_base()) {
      geometry_msgs::msg::WrenchStamped w;
      w.header.stamp = node_ptr_->now();
      w.header.frame_id = "end_effector_link"; // Assumes this frame exists
      w.wrench.force.x  = feedback.base().tool_external_wrench_force_x();
      w.wrench.force.y  = feedback.base().tool_external_wrench_force_y();
      w.wrench.force.z  = feedback.base().tool_external_wrench_force_z();
      w.wrench.torque.x = feedback.base().tool_external_wrench_torque_x();
      w.wrench.torque.y = feedback.base().tool_external_wrench_torque_y();
      w.wrench.torque.z = feedback.base().tool_external_wrench_torque_z();
      wrench_publisher_->publish(w);
    }

    // Process any pending ROS callbacks (e.g., for services).
    rclcpp::spin_some(node_ptr_);
  }
  catch (const Kinova::Api::KDetailedException &ex) {
    in_fault_ = true;
    RCLCPP_ERROR(LOGGER, "Robot feedback error: %s", ex.what());
    RCLCPP_ERROR(LOGGER, "To recover, call the /kortex3_hardware/clear_faults service.");
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type Kortex3HardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Safety guard: Do not send commands if the arm is in fault.
  if (in_fault_) {
    return hardware_interface::return_type::OK;
  }

  // Safety guard: Only send velocity commands if in the correct operating mode.
  if (last_operating_mode_ != Kinova::Api::Common::OPERATING_MODE_AUTO) {
    return hardware_interface::return_type::OK;
  }

  // Safety guard: Block velocity commands while program is running or within 1 second after completion.
  // Check if program is currently active
  if (is_program_active(last_program_status_)) {
    RCLCPP_DEBUG_THROTTLE(LOGGER, *node_ptr_->get_clock(), 1000,
                          "Velocity commands blocked: program is currently running (status: %d)",
                          last_program_status_);
    return hardware_interface::return_type::OK;
  }

  // Check if we're within 1 second after program completion
  auto now = std::chrono::steady_clock::now();
  auto time_since_program_end = std::chrono::duration_cast<std::chrono::milliseconds>(
    now - program_end_time_).count();

  if (time_since_program_end >= 0 && time_since_program_end < 1000) {
    RCLCPP_DEBUG_THROTTLE(LOGGER, *node_ptr_->get_clock(), 1000,
                          "Velocity commands blocked: %ld ms since program ended (blocking for 1 second)",
                          time_since_program_end);
    return hardware_interface::return_type::OK;
  }

  try {
    Kinova::Api::Base::Action action;
    action.set_name("ros2_control_velocity_command");
    auto *js = action.mutable_send_joint_speeds();

    // Convert joint velocities from rad/s (ROS) to deg/s (Kortex API).
    for (size_t i = 0; i < actuator_count_; ++i) {
      auto &sp = *js->add_joint_speeds();
      sp.set_joint_identifier(i);
      sp.set_value(static_cast<float>(joint_velocities_cmd_[i] * 180.0 / M_PI));
    }
    base_mqtt_->ExecuteAction(action);
  }
  catch (const Kinova::Api::KDetailedException &e) {
    std::string error_msg = e.what();

    // Check if error is INVALID_PARAM - this usually means robot is in fault state
    // but read() hasn't detected it yet (race condition)
    if (error_msg.find("INVALID_PARAM") != std::string::npos ||
        error_msg.find("ROBOT_IN_FAULT") != std::string::npos ||
        error_msg.find("WRONG_MODE") != std::string::npos) {
      // Robot likely in fault state - set flag and let read() handle proper detection/logging
      RCLCPP_DEBUG(LOGGER, "Command rejected - robot likely in fault. Will be detected in read().");
      in_fault_ = true;
      return hardware_interface::return_type::OK;  // Return OK, not ERROR - this is expected
    } else {
      // Unexpected error - log as ERROR
      RCLCPP_ERROR(LOGGER, "Unexpected fault during write(): %s", e.what());
      RCLCPP_ERROR(LOGGER, "To recover, call the /kortex3_hardware/clear_faults service.");
      in_fault_ = true;
      return hardware_interface::return_type::ERROR;
    }
  }

  // Send gripper commands
  if (use_internal_bus_gripper_comm_ && !gripper_name_.empty())
  {
    if (!gripper_a_.joint_name_.empty())
    {
      gripper_a_.sendCommand(gripper_a_.command_position_, gripper_mtx_);
    }
    if (!gripper_b_.joint_name_.empty())
    {
      gripper_b_.sendCommand(gripper_b_.command_position_, gripper_mtx_);
    }
  }

  return hardware_interface::return_type::OK;
}

void Kortex3HardwareInterface::check_and_power_on_robot()
{
  auto arm_state = base_mqtt_->GetArmState();
  if (arm_state.active_state() == k_api::Common::ArmState::ARMSTATE_ARM_OPERATIONAL)
  {
    RCLCPP_INFO(LOGGER, "Robot arm is already operational.");
    return;
  }

  RCLCPP_WARN(LOGGER,
    "Robot arm is not operational (state: %s). Attempting to activate...",
    k_api::Common::ArmState_Name(arm_state.active_state()).c_str());
  try
  {
    base_mqtt_->ActivateRobot();
    RCLCPP_INFO(LOGGER, "Power-on command sent. Waiting for arm to become operational (timeout: 60s)...");

    const auto timeout = std::chrono::seconds(60);
    auto start_time = std::chrono::steady_clock::now();
    bool is_operational = false;

    while (std::chrono::steady_clock::now() - start_time < timeout)
    {
      auto current_state = base_mqtt_->GetArmState().active_state();
      if (current_state == k_api::Common::ArmState::ARMSTATE_ARM_OPERATIONAL)
      {
        RCLCPP_INFO(LOGGER, "Robot arm is now operational.");
        is_operational = true;
        break;
      }
      if (current_state == k_api::Common::ArmState::ARMSTATE_IN_FAULT)
      {
        RCLCPP_ERROR(LOGGER, "Robot arm entered a fault state during initialization.");
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    if (!is_operational)
    {
      std::string error_msg = "Failed to power on the robot arm within the timeout. Final state: "
        + k_api::Common::ArmState_Name(base_mqtt_->GetArmState().active_state());
      RCLCPP_ERROR(LOGGER, error_msg.c_str());
      throw std::runtime_error(error_msg);
    }
  }
  catch (const k_api::KDetailedException &ex)
  {
    RCLCPP_ERROR(LOGGER, "Kortex API exception while trying to activate robot: %s", ex.what());
    throw;
  }
}

void Kortex3HardwareInterface::change_operating_mode(
  const k_api::Common::OperatingModeType &mode)
{
  try
  {
    k_api::Common::ModeSelection mode_selection;
    mode_selection.set_operating_mode(mode);
    base_mqtt_->SelectOperatingMode(mode_selection);
    // Allow time for the controller to switch modes.
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  }
  catch (const k_api::KDetailedException &ex)
  {
    RCLCPP_ERROR(LOGGER, "Failed to change operating mode: %s", ex.what());
  }
}

void Kortex3HardwareInterface::send_zero_velocities()
{
  try
  {
    k_api::Base::Action action;
    action.set_name("ros2_control_stop");
    auto joint_speeds = action.mutable_send_joint_speeds();
    for (size_t i = 0; i < actuator_count_; i++)
    {
      auto speed = joint_speeds->add_joint_speeds();
      speed->set_joint_identifier(i);
      speed->set_value(0.0f);
    }
    base_mqtt_->ExecuteAction(action);
  }
  catch (const k_api::KDetailedException &ex)
  {
    RCLCPP_ERROR(LOGGER, "Failed to send zero velocities: %s", ex.what());
  }
}

bool Kortex3HardwareInterface::is_program_active(k_api::ProgramRunner::Status status) const
{
  return (status == k_api::ProgramRunner::STATUS_RUNNING ||
          status == k_api::ProgramRunner::STATUS_STARTING ||
          status == k_api::ProgramRunner::STATUS_PAUSED ||
          status == k_api::ProgramRunner::STATUS_PAUSED_AUTOMATIC_RESUME ||
          status == k_api::ProgramRunner::STATUS_WAITING_FOR_ACKNOWLEDGE ||
          status == k_api::ProgramRunner::STATUS_STOPPING);
}

bool Kortex3HardwareInterface::check_unsafe_controllers_active(std::string& error_message)
{
  // List of controllers that should block program execution
  const std::vector<std::string> unsafe_controllers = {
    "joint_velocity_controller",
    "cartesian_motion_controller",
    "motion_control_handle"
  };

// Perform the service call in a separate thread to avoid executor conflicts
std::atomic<bool> call_completed{false};
std::shared_ptr<controller_manager_msgs::srv::ListControllers::Response> response;
std::string thread_error;

  std::thread service_thread([this, &call_completed, &response, &thread_error]() {
    try {
      // Create a separate node for this thread to avoid executor conflicts
      auto temp_node = std::make_shared<rclcpp::Node>("active_motion_controllers_checker_temp");
      auto temp_client = temp_node->create_client<controller_manager_msgs::srv::ListControllers>(
          "/controller_manager/list_controllers");

      // Wait for service to be available
      if (!temp_client->wait_for_service(std::chrono::milliseconds(500))) {
        thread_error = "Controller manager service not available";
        call_completed = true;
        return;
      }

      // Call the service
      auto request = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
      auto future = temp_client->async_send_request(request);

      // Spin until complete
      rclcpp::executors::SingleThreadedExecutor executor;
      executor.add_node(temp_node);

      auto ret_code = executor.spin_until_future_complete(future, std::chrono::seconds(1));

      if (ret_code == rclcpp::FutureReturnCode::SUCCESS) {
        response = future.get();
      } else {
        thread_error = "Service call timed out";
      }
    } catch (const std::exception& ex) {
      thread_error = std::string("Exception: ") + ex.what();
    }
    call_completed = true;
  });

  // Wait for the thread to complete with timeout
  auto start_time = std::chrono::steady_clock::now();
  while (!call_completed &&
         (std::chrono::steady_clock::now() - start_time) < std::chrono::seconds(2)) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  // Join the thread
  if (service_thread.joinable()) {
    service_thread.join();
  }

  // Check for errors
  if (!call_completed || !response) {
    error_message = thread_error.empty() ?
        "Failed to get controller list from controller manager (timeout)." : thread_error;
    RCLCPP_DEBUG(LOGGER, "%s", error_message.c_str());
    return false;  // Allow program execution if we can't check (fail-safe)
  }
  std::vector<std::string> active_unsafe_controllers;

  // Check each controller in the response
  for (const auto& controller : response->controller) {
    // Check if this controller is in the unsafe list and is active
    if (std::find(unsafe_controllers.begin(), unsafe_controllers.end(), controller.name) != unsafe_controllers.end()) {
      if (controller.state == "active") {
        active_unsafe_controllers.push_back(controller.name);
      }
    }
  }

  // If any unsafe controllers are active, build error message and return true
  if (!active_unsafe_controllers.empty()) {
    error_message = "Cannot execute program: The following motion controllers are active: ";
    for (size_t i = 0; i < active_unsafe_controllers.size(); ++i) {
      error_message += active_unsafe_controllers[i];
      if (i < active_unsafe_controllers.size() - 1) {
        error_message += ", ";
      }
    }
    error_message += ". Please stop these controllers before running a program.";
    return true;  // Unsafe controllers are active
  }

  return false;  // Safe to execute program
}

std::vector<std::string> Kortex3HardwareInterface::get_active_motion_controllers()
{
  // List of motion controllers we care about
  const std::vector<std::string> motion_controllers = {
    "joint_velocity_controller",
    "cartesian_motion_controller",
    "motion_control_handle",
    "joint_trajectory_controller"
  };

  std::vector<std::string> active_controllers;

  // Perform the service call in a separate thread to avoid executor conflicts
  std::atomic<bool> call_completed{false};
  std::shared_ptr<controller_manager_msgs::srv::ListControllers::Response> response;
  std::string thread_error;

  std::thread service_thread([this, &call_completed, &response, &thread_error]() {
    try {
      // Create a separate node for this thread to avoid executor conflicts
      auto temp_node = std::make_shared<rclcpp::Node>("active_motion_controllers_lister_temp");
      auto temp_client = temp_node->create_client<controller_manager_msgs::srv::ListControllers>(
          "/controller_manager/list_controllers");

      // Wait for service to be available
      if (!temp_client->wait_for_service(std::chrono::milliseconds(500))) {
        thread_error = "Controller manager service not available";
        std::cout << "Controller manager service not available" << std::endl;
        call_completed = true;
        return;
      }

      // Call the service
      auto request = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
      auto future = temp_client->async_send_request(request);

      // Spin until complete
      rclcpp::executors::SingleThreadedExecutor executor;
      executor.add_node(temp_node);

      auto ret_code = executor.spin_until_future_complete(future, std::chrono::seconds(1));

      if (ret_code == rclcpp::FutureReturnCode::SUCCESS) {
        response = future.get();
      } else {
        thread_error = "Service call timed out";
      }
    } catch (const std::exception& ex) {
      thread_error = std::string("Exception: ") + ex.what();
    }
    call_completed = true;
  });

  // Wait for the thread to complete with timeout
  auto start_time = std::chrono::steady_clock::now();
  while (!call_completed &&
         (std::chrono::steady_clock::now() - start_time) < std::chrono::seconds(2)) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  // Join the thread
  if (service_thread.joinable()) {
    service_thread.join();
  }

  // Check for errors
  if (!call_completed || !response) {
    RCLCPP_WARN(LOGGER, "Failed to get controller list: %s",
                thread_error.empty() ? "timeout" : thread_error.c_str());
    return active_controllers;  // Return empty list
  }

  // Check each controller in the response
  for (const auto& controller : response->controller) {
    // Check if this controller is in our motion controllers list and is active
    if (std::find(motion_controllers.begin(), motion_controllers.end(), controller.name) != motion_controllers.end()) {
      if (controller.state == "active") {
        active_controllers.push_back(controller.name);
        RCLCPP_DEBUG(LOGGER, "Found active motion controller: %s", controller.name.c_str());
      }
    }
  }

  return active_controllers;
}

void Kortex3HardwareInterface::handle_set_operating_mode(
  const std::shared_ptr<kortex3_hardware::srv::SetOperatingMode::Request> request,
  std::shared_ptr<kortex3_hardware::srv::SetOperatingMode::Response> response)
{
  RCLCPP_INFO(LOGGER, "Received SetOperatingMode request for mode ID: %d", request->operating_mode);

  // Map the service request's uint8 to the Kortex API enum.
  k_api::Common::OperatingModeType requested_mode;
  switch (request->operating_mode)
  {
    case 0: requested_mode = k_api::Common::OPERATING_MODE_UNSPECIFIED;  break;
    case 1: requested_mode = k_api::Common::OPERATING_MODE_JOG_MANUAL;   break;
    case 2: requested_mode = k_api::Common::OPERATING_MODE_HAND_GUIDING; break;
    case 3: requested_mode = k_api::Common::OPERATING_MODE_HOLD_TO_RUN;  break;
    case 4: requested_mode = k_api::Common::OPERATING_MODE_AUTO;         break;
    case 5: requested_mode = k_api::Common::OPERATING_MODE_MONITORED_STOP; break;
    default:
      RCLCPP_ERROR(LOGGER, "Invalid operating mode requested: %d", request->operating_mode);
      response->success = false;
      response->message = "Invalid mode (must be 0-5).";
      return;
  }

  try
  {
    change_operating_mode(requested_mode);
    last_operating_mode_ = requested_mode; // Update internal state for write() gating.

    response->success = true;
    response->message = "Operating mode set to " + k_api::Common::OperatingModeType_Name(requested_mode);
    RCLCPP_INFO(LOGGER, "Successfully set operating mode to: %s",
      k_api::Common::OperatingModeType_Name(requested_mode).c_str());
  }
  catch (const k_api::KDetailedException & ex)
  {
    response->success = false;
    response->message = std::string("Kortex API error: ") + ex.what();
    RCLCPP_ERROR(LOGGER, "Failed to set operating mode: %s", ex.what());
  }
  catch (const std::exception & ex)
  {
    response->success = false;
    response->message = std::string("Error: ") + ex.what();
    RCLCPP_ERROR(LOGGER, "Failed to set operating mode: %s", ex.what());
  }
}

void Kortex3HardwareInterface::handle_clear_faults(
  const std::shared_ptr<kortex3_hardware::srv::ClearFaults::Request> /*request*/,
  std::shared_ptr<kortex3_hardware::srv::ClearFaults::Response> response)
{
  RCLCPP_INFO(LOGGER, "Received ClearFaults request.");

  // STEP 1: Get list of active controllers BEFORE clearing fault
  std::cout << "\n=== Section 1: Get Active Controllers (before recovery) ===" << std::endl;
  auto active_controllers = get_active_motion_controllers();
  if (active_controllers.empty()) {
    std::cout << "No active motion controllers found." << std::endl;
  } else {
    std::cout << "Found " << active_controllers.size() << " active motion controller(s):" << std::endl;
    for (const auto& controller : active_controllers) {
      std::cout << "  - " << controller << std::endl;
    }
  }

  try {
    auto armState = base_mqtt_->GetArmState();
    if (armState.active_state() == k_api::Common::ARMSTATE_IN_FAULT ||
        armState.active_state() == k_api::Common::ARMSTATE_IN_FAULT_POWERED_OFF)
    {
      // 1. Clear the fault, which puts the arm into RECOVERY state.
      base_mqtt_->ClearFaults();
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));

      // Re-fetch state after ClearFaults()
      armState = base_mqtt_->GetArmState();
      RCLCPP_INFO(LOGGER, "Called ClearFaults(). Arm state: %s",
                  k_api::Common::ArmState_Name(armState.active_state()).c_str());

      // 2. Handle recovery based on arm state after ClearFaults()
      if (armState.active_state() == k_api::Common::ARMSTATE_RECOVERY)
      {
        // Arm is in RECOVERY - requires manual hand-guiding
        RCLCPP_WARN(LOGGER, "═══════════════════════════════════════════════════════════");
        RCLCPP_WARN(LOGGER, "ROBOT IS NOW IN RECOVERY STATE");
        RCLCPP_WARN(LOGGER, "If the fault was due to:");
        RCLCPP_WARN(LOGGER, "  - Protection zone violation");
        RCLCPP_WARN(LOGGER, "  - Proximity to position limits");
        RCLCPP_WARN(LOGGER, "  - Singularity");
        RCLCPP_WARN(LOGGER, "MANUAL ACTION REQUIRED:");
        RCLCPP_WARN(LOGGER, "  1. Press the hand guiding button on the robot arm");
        RCLCPP_WARN(LOGGER, "  2. Manually guide the robot to a safe position");
        RCLCPP_WARN(LOGGER, "  3. Release the hand guiding button");
        RCLCPP_WARN(LOGGER, "═══════════════════════════════════════════════════════════");

        // Open /dev/tty directly to read from the controlling terminal
        std::ifstream tty("/dev/tty");
        if (tty.is_open())
        {
          std::cerr << "\nPress ENTER to proceed with recovery after hand-guiding..." << std::endl;
          std::string dummy;
          std::getline(tty, dummy);
          tty.close();
          RCLCPP_INFO(LOGGER, "User confirmed. Proceeding with recovery...");
        }
        else
        {
          RCLCPP_ERROR(LOGGER, "Failed to open /dev/tty for user input. Proceeding automatically after 5 seconds...");
          std::this_thread::sleep_for(std::chrono::milliseconds(5000));
        }

        // 3. Exit recovery state to return to OPERATIONAL
        base_mqtt_->ExitRecoveryState();
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        RCLCPP_INFO(LOGGER, "Called ExitRecoveryState(). Waiting for arm to become OPERATIONAL...");

        // 4. Wait for the arm to confirm it is operational
        const auto timeout = std::chrono::seconds(10);
        auto start = std::chrono::steady_clock::now();
        bool became_operational = false;
        while (std::chrono::steady_clock::now() - start < timeout) {
          if (base_mqtt_->GetArmState().active_state() == k_api::Common::ARMSTATE_ARM_OPERATIONAL) {
            RCLCPP_INFO(LOGGER, "Arm recovered to OPERATIONAL.");
            became_operational = true;
            break;
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }

        // 5. Report post recovery result
        if (became_operational)
        {
          in_fault_ = false;
          fault_recently_logged_ = false;

          // Set robot back to AUTO mode
          //RCLCPP_INFO(LOGGER, "Setting robot to AUTO mode...");
          change_operating_mode(k_api::Common::OPERATING_MODE_MONITORED_STOP);
          last_operating_mode_ = k_api::Common::OPERATING_MODE_MONITORED_STOP;

          // STEP 2: Schedule controller deactivation to happen in read()
          // We CANNOT call controller_manager from here due to executor deadlock
          if (!active_controllers.empty()) {
            std::cout << "\n=== Section 2: Schedule Controller Deactivation ===" << std::endl;
            std::cout << "Scheduling deactivation of " << active_controllers.size() << " controller(s)..." << std::endl;

            // Store the controllers to deactivate and set the flag
            controllers_to_deactivate_ = active_controllers;
            pending_controller_deactivation_ = true;

            std::cout << "Controllers will be deactivated in the next read() cycle." << std::endl;
            RCLCPP_INFO(LOGGER, "Scheduled deactivation of %zu motion controller(s).", active_controllers.size());
            std::cout << "=== END Section 2 ===" << std::endl;
          } else {
            RCLCPP_INFO(LOGGER, "No motion controllers to deactivate.");
          }

          response->success = true;
          response->message = "Faults cleared and arm recovered to OPERATIONAL.";
        }
        else
        {
          response->success = false;
          response->message = "Recovery sequence timeout - arm did not reach OPERATIONAL state. "
                             "Hand guiding may not have been performed properly. "
                             "Current state: " + k_api::Common::ArmState_Name(base_mqtt_->GetArmState().active_state());
          RCLCPP_ERROR(LOGGER, "Failed to exit recovery. Arm state: %s",
                       k_api::Common::ArmState_Name(base_mqtt_->GetArmState().active_state()).c_str());
        }
      }
      else if (armState.active_state() == k_api::Common::ARMSTATE_ARM_OPERATIONAL)
      {
        // Fault cleared and arm is already operational - no recovery needed
        RCLCPP_INFO(LOGGER, "Fault cleared. Arm is already OPERATIONAL.");
        in_fault_ = false;
        fault_recently_logged_ = false;

        // Set robot back to AUTO mode
        RCLCPP_INFO(LOGGER, "Setting robot to AUTO mode...");
        change_operating_mode(k_api::Common::OPERATING_MODE_AUTO);
        last_operating_mode_ = k_api::Common::OPERATING_MODE_AUTO;

        response->success = true;
        response->message = "Faults cleared. Arm is OPERATIONAL.";
      }
      else
      {
        // Arm is in an unexpected state after clearing faults
        RCLCPP_WARN(LOGGER, "Fault cleared but arm is in unexpected state: %s",
                    k_api::Common::ArmState_Name(armState.active_state()).c_str());
        in_fault_ = false;  // Reset fault flag anyway
        fault_recently_logged_ = false;
        response->success = true;
        response->message = "Faults cleared. Arm state: " + k_api::Common::ArmState_Name(armState.active_state());
      }
    }
    else {
      response->success = false;
      response->message = "Robot is not in a fault state. Current state: " +
        k_api::Common::ArmState_Name(armState.active_state());
      RCLCPP_WARN(LOGGER, "ClearFaults requested, but robot is not in fault. State: %s",
        k_api::Common::ArmState_Name(armState.active_state()).c_str());
    }
  }
  catch (const k_api::KDetailedException &ex) {
    response->success = false;
    response->message = "Kortex API error: " + std::string(ex.what());
    RCLCPP_ERROR(LOGGER, "Failed to clear faults: %s", ex.what());
  }
}

void Kortex3HardwareInterface::handle_simulate_estop(
  const std::shared_ptr<kortex3_hardware::srv::SimulateEstop::Request> request,
  std::shared_ptr<kortex3_hardware::srv::SimulateEstop::Response> response)
{
  if (request->enable) {
    RCLCPP_WARN(LOGGER, "==================================================");
    RCLCPP_WARN(LOGGER, "TRIGGERING ROBOT FAULT VIA SAFETY VIOLATION");
    RCLCPP_WARN(LOGGER, "Method: Excessive joint velocity command");
    RCLCPP_WARN(LOGGER, "==================================================");

    try {
      // Create action with excessive joint velocities that violate safety limits
      Kinova::Api::Base::Action action;
      action.set_name("fault_injection_test");
      auto *js = action.mutable_send_joint_speeds();

      // Send excessive velocity to joint 6 only to trigger safety violation
      // Normal safe limits: ~1-2 rad/s (57-115 deg/s)
      // We send 180 deg/s (~3.14 rad/s) to joint 6
      const float excessive_velocity = 320.0f;  // deg/s
      const size_t target_joint = 4;  // Joint 6 (0-indexed)

      auto &sp = *js->add_joint_speeds();
      sp.set_joint_identifier(target_joint);
      sp.set_value(excessive_velocity);

      RCLCPP_WARN(LOGGER, "Sending excessive velocity (%.0f deg/s, ~%.1f rad/s) to joint %zu",
                  excessive_velocity, excessive_velocity * M_PI / 180.0, target_joint + 1);

      // Execute - robot's safety system should reject this
      base_mqtt_->ExecuteAction(action);

      // Wait briefly for fault detection in the read() loop
      std::this_thread::sleep_for(std::chrono::milliseconds(500));

      // Verify robot entered fault state
      auto arm_state = base_mqtt_->GetArmState();
      if (arm_state.active_state() == k_api::Common::ARMSTATE_IN_FAULT ||
          arm_state.active_state() == k_api::Common::ARMSTATE_IN_FAULT_POWERED_OFF) {
        response->success = true;
        response->message = "SUCCESS: Robot entered FAULT state (" +
                          k_api::Common::ArmState_Name(arm_state.active_state()) +
                          "). To recover, use the teach pendant to clear faults and turn on the arm";
        RCLCPP_WARN(LOGGER, "Robot successfully entered fault state: %s",
                    k_api::Common::ArmState_Name(arm_state.active_state()).c_str());
        RCLCPP_WARN(LOGGER, "To recover: Use the teach pendant to cleat faults and turn on the arm");
      } else {
        response->success = false;
        response->message = "Command sent but robot did not enter fault state. "
                          "Current state: " + k_api::Common::ArmState_Name(arm_state.active_state()) +
                          ". The robot's safety system may have rejected the command without entering fault.";
        RCLCPP_WARN(LOGGER, "Robot did not enter fault state. Current state: %s",
                    k_api::Common::ArmState_Name(arm_state.active_state()).c_str());
      }

    } catch (const Kinova::Api::KDetailedException &e) {
      // API rejection is expected - the robot's safety system rejects invalid commands
      RCLCPP_INFO(LOGGER, "Kortex API rejected command (expected behavior): %s", e.what());

      // Check if rejection triggered a fault state
      try {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        auto arm_state = base_mqtt_->GetArmState();
        if (arm_state.active_state() == k_api::Common::ARMSTATE_IN_FAULT ||
            arm_state.active_state() == k_api::Common::ARMSTATE_IN_FAULT_POWERED_OFF) {
          response->success = true;
          response->message = "SUCCESS: API rejected command and robot entered FAULT state (" +
                            k_api::Common::ArmState_Name(arm_state.active_state()) +
                            "). Call clear_faults service to recover.";
          RCLCPP_WARN(LOGGER, "Fault successfully triggered via API rejection");
        } else {
          response->success = false;
          response->message = "API rejected command but robot not in fault. "
                            "Current state: " + k_api::Common::ArmState_Name(arm_state.active_state()) +
                            ". Try pressing physical E-stop instead.";
          RCLCPP_INFO(LOGGER, "Command rejected but no fault state entered");
        }
      } catch (...) {
        response->success = false;
        response->message = "API rejected command. Unable to verify robot state.";
        RCLCPP_ERROR(LOGGER, "Failed to verify robot state after command rejection");
      }
    } catch (const std::exception &e) {
      response->success = false;
      response->message = std::string("Unexpected error during fault injection: ") + e.what();
      RCLCPP_ERROR(LOGGER, "Unexpected error during fault injection: %s", e.what());
    }

  } else {
    // Disable - just provide guidance
    RCLCPP_INFO(LOGGER, "Fault injection disabled. Use clear_faults service to recover from any active faults.");
    response->success = true;
    response->message = "To clear active faults, call: ros2 service call /kortex3_hardware/clear_faults kortex3_hardware/srv/ClearFaults \"{}\"";
  }
}

void Kortex3HardwareInterface::handle_run_program(
  const std::shared_ptr<kortex3_hardware::srv::RunProgram::Request> request,
  std::shared_ptr<kortex3_hardware::srv::RunProgram::Response> response)
{
  RCLCPP_INFO(LOGGER, "Received RunProgram request for program name: '%s'", request->program_name.c_str());

  // Validate program name is not empty
  if (request->program_name.empty()) {
    response->success = false;
    response->message = "Program name cannot be empty.";
    RCLCPP_ERROR(LOGGER, "%s", response->message.c_str());
    return;
  }

  // Safety check: Verify no unsafe controllers are active
  std::string error_message;
  if (check_unsafe_controllers_active(error_message)) {
    response->success = false;
    response->message = error_message;
    RCLCPP_ERROR(LOGGER, "%s", error_message.c_str());
    return;
  }

  try {
    // Get all programs to find the matching program by name
    auto program_list = program_runner_->ReadAllPrograms();

    uint32_t program_id = 0;
    bool found = false;

    for (int i = 0; i < program_list.programs_size(); ++i) {
      const auto& program = program_list.programs(i);
      if (program.name() == request->program_name) {
        program_id = program.handle().identifier();
        found = true;
        break;
      }
    }

    if (!found) {
      response->success = false;
      response->message = "Program '" + request->program_name + "' not found. Use list_programs service to see available programs.";
      RCLCPP_ERROR(LOGGER, "%s", response->message.c_str());
      return;
    }

    // Create the program handle
    k_api::Common::ProgramHandle program_handle;
    program_handle.set_identifier(program_id);
    program_handle.set_permission(0);  // Default permission

    // Create the runnable handle
    k_api::ProgramRunner::RunnableHandle runnable_handle;
    runnable_handle.mutable_program_handle()->CopyFrom(program_handle);

    // Create the start configuration
    k_api::ProgramRunner::ProgramStartConfiguration start_config;
    start_config.mutable_handle()->CopyFrom(runnable_handle);
    start_config.set_debug_mode(false);  // Debug mode disabled

    // Start the program
    program_runner_->Start(start_config);

    response->success = true;
    response->message = "Program '" + request->program_name + "' (ID: " + std::to_string(program_id) + ") started successfully.";
    RCLCPP_INFO(LOGGER, "%s", response->message.c_str());
  }
  catch (const k_api::KDetailedException &ex) {
    response->success = false;
    response->message = "Failed to start program: " + std::string(ex.what());
    RCLCPP_ERROR(LOGGER, "%s", response->message.c_str());
  }
  catch (const std::exception &ex) {
    response->success = false;
    response->message = "Unexpected error: " + std::string(ex.what());
    RCLCPP_ERROR(LOGGER, "%s", response->message.c_str());
  }
}

void Kortex3HardwareInterface::handle_list_programs(
  const std::shared_ptr<kortex3_hardware::srv::ListPrograms::Request> /*request*/,
  std::shared_ptr<kortex3_hardware::srv::ListPrograms::Response> response)
{
  RCLCPP_INFO(LOGGER, "Received ListPrograms request.");

  try {
    // Get all programs from the robot
    auto program_list = program_runner_->ReadAllPrograms();

    response->programs.clear();
    for (int i = 0; i < program_list.programs_size(); ++i) {
      const auto& program = program_list.programs(i);

      kortex3_hardware::msg::ProgramInfo program_info;
      program_info.identifier = program.handle().identifier();
      program_info.name = program.name();

      response->programs.push_back(program_info);
    }

    response->success = true;
    response->message = "Found " + std::to_string(response->programs.size()) + " program(s).";
    RCLCPP_INFO(LOGGER, "%s", response->message.c_str());
  }
  catch (const k_api::KDetailedException &ex) {
    response->success = false;
    response->message = "Failed to list programs: " + std::string(ex.what());
    RCLCPP_ERROR(LOGGER, "%s", response->message.c_str());
  }
  catch (const std::exception &ex) {
    response->success = false;
    response->message = "Unexpected error: " + std::string(ex.what());
    RCLCPP_ERROR(LOGGER, "%s", response->message.c_str());
  }
}

void Kortex3HardwareInterface::handle_stop_program(
  const std::shared_ptr<kortex3_hardware::srv::StopProgram::Request> /*request*/,
  std::shared_ptr<kortex3_hardware::srv::StopProgram::Response> response)
{
  RCLCPP_INFO(LOGGER, "Received StopProgram request.");

  try {
    // Stop the program
    program_runner_->Stop();
    RCLCPP_INFO(LOGGER, "Program stopped.");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    // Switch operating mode back to AUTO
    RCLCPP_INFO(LOGGER, "Switching operating mode back to AUTO...");
    change_operating_mode(k_api::Common::OPERATING_MODE_AUTO);
    last_operating_mode_ = k_api::Common::OPERATING_MODE_AUTO;
    RCLCPP_INFO(LOGGER, "Operating mode set to AUTO.");

    response->success = true;
    response->message = "Program stopped successfully and operating mode set to AUTO.";
    RCLCPP_INFO(LOGGER, "%s", response->message.c_str());
  }
  catch (const k_api::KDetailedException &ex) {
    response->success = false;
    response->message = "Failed to stop program: " + std::string(ex.what());
    RCLCPP_ERROR(LOGGER, "%s", response->message.c_str());
  }
  catch (const std::exception &ex) {
    response->success = false;
    response->message = "Unexpected error: " + std::string(ex.what());
    RCLCPP_ERROR(LOGGER, "%s", response->message.c_str());
  }
}

void Kortex3HardwareInterface::handle_get_program_status(
  const std::shared_ptr<kortex3_hardware::srv::GetProgramStatus::Request> /*request*/,
  std::shared_ptr<kortex3_hardware::srv::GetProgramStatus::Response> response)
{
  RCLCPP_INFO(LOGGER, "Received GetProgramStatus request.");

  try {
    auto status_info = program_runner_->GetStatus();

    // Convert status enum to string
    std::string status_str;
    switch (status_info.status()) {
      case k_api::ProgramRunner::STATUS_IDLE:
        status_str = "IDLE";
        break;
      case k_api::ProgramRunner::STATUS_RUNNING:
        status_str = "RUNNING";
        break;
      case k_api::ProgramRunner::STATUS_PAUSED:
        status_str = "PAUSED";
        break;
      case k_api::ProgramRunner::STATUS_STARTING:
        status_str = "STARTING";
        break;
      case k_api::ProgramRunner::STATUS_STOPPING:
        status_str = "STOPPING";
        break;
      case k_api::ProgramRunner::STATUS_PAUSED_AUTOMATIC_RESUME:
        status_str = "PAUSED_AUTOMATIC_RESUME";
        break;
      case k_api::ProgramRunner::STATUS_WAITING_FOR_ACKNOWLEDGE:
        status_str = "WAITING_FOR_ACKNOWLEDGE";
        break;
      default:
        status_str = "UNSPECIFIED";
        break;
    }

    response->success = true;
    response->status = status_str;
    response->message = "Program runner status: " + status_str;
    RCLCPP_INFO(LOGGER, "%s", response->message.c_str());
  }
  catch (const k_api::KDetailedException &ex) {
    response->success = false;
    response->message = "Failed to get program status: " + std::string(ex.what());
    RCLCPP_ERROR(LOGGER, "%s", response->message.c_str());
  }
  catch (const std::exception &ex) {
    response->success = false;
    response->message = "Unexpected error: " + std::string(ex.what());
    RCLCPP_ERROR(LOGGER, "%s", response->message.c_str());
  }
}

void Kortex3HardwareInterface::handle_list_protection_zones(
  const std::shared_ptr<kortex3_hardware::srv::ListProtectionZones::Request> /*request*/,
  std::shared_ptr<kortex3_hardware::srv::ListProtectionZones::Response> response)
{
  RCLCPP_INFO(LOGGER, "Received ListProtectionZones request.");

  try {
    // Get all protection zones from the robot
    auto zone_list = protection_zone_->ReadAllProtectionZones();

    response->zones.clear();
    for (int i = 0; i < zone_list.protection_zones_size(); ++i) {
      const auto& zone = zone_list.protection_zones(i);

      kortex3_hardware::msg::ProtectionZoneInfo zone_info;
      zone_info.identifier = zone.handle().identifier();
      zone_info.name = zone.name();
      zone_info.is_enabled = zone.is_enabled();

      response->zones.push_back(zone_info);
    }

    response->success = true;
    response->message = "Found " + std::to_string(response->zones.size()) + " protection zone(s).";
    RCLCPP_INFO(LOGGER, "%s", response->message.c_str());

    // Log details of each zone
    for (const auto& zone : response->zones) {
      RCLCPP_INFO(LOGGER, "  Zone ID %u: %s [%s]",
                  zone.identifier,
                  zone.name.c_str(),
                  zone.is_enabled ? "ENABLED" : "DISABLED");
    }
  }
  catch (const k_api::KDetailedException &ex) {
    response->success = false;
    response->message = "Failed to list protection zones: " + std::string(ex.what());
    RCLCPP_ERROR(LOGGER, "%s", response->message.c_str());
  }
  catch (const std::exception &ex) {
    response->success = false;
    response->message = "Unexpected error: " + std::string(ex.what());
    RCLCPP_ERROR(LOGGER, "%s", response->message.c_str());
  }
}

} // namespace kortex3_driver

#include "pluginlib/class_list_macros.hpp"
// Registers this class with pluginlib, making it available to the ros2_control controller manager.
PLUGINLIB_EXPORT_CLASS(
  kortex3_driver::Kortex3HardwareInterface,
  hardware_interface::SystemInterface)