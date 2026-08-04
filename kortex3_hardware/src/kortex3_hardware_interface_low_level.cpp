
#include <chrono>
#include <cmath>
#include <string>
#include <thread>
#include <vector>

#include "kortex3_hardware/kortex3_hardware_interface_low_level.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace kortex3_driver
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("Kortex3HardwareInterfaceLowLevel");

Kortex3HardwareInterfaceLowLevel::Kortex3HardwareInterfaceLowLevel()
  : mode_selection_(k_api::Common::ModeSelection())
  , servoing_mode_info_(k_api::Base::ServoingModeInformation())
  , safety_system_(k_api::SafetyFunctions::SafetySystem())
  , actuator_count_(6)  // Default, updated from robot during activation.
  , k_api_twist_(nullptr)
  , stop_low_level_control_mode_(false)
  , stop_joint_velocity_control_mode_(false)
  , stop_twist_control_mode_(false)
  , start_low_level_control_mode_(false)
  , start_joint_velocity_control_mode_(false)
  , start_twist_control_mode_(false)
  , low_level_control_mode_running_(false)
  , joint_velocity_control_mode_running_(false)
  , twist_control_mode_running_(false)
  , base_command_frame_id_(0)
  , low_level_operating_mode_(k_api::Common::OPERATING_MODE_AUTO)
  , low_level_safety_mode_(k_api::SafetyFunctions::SAFETY_SYSTEM_MODE_REDUCED)
  , use_internal_bus_gripper_comm_(false)
  , gripper_a_(9)
  , gripper_b_(10)
  , gripper_a_pos_(std::numeric_limits<double>::quiet_NaN())
  , gripper_a_vel_(0.0)
  , gripper_a_cmd_(std::numeric_limits<double>::quiet_NaN())
  , gripper_b_pos_(std::numeric_limits<double>::quiet_NaN())
  , gripper_b_vel_(0.0)
  , gripper_b_cmd_(std::numeric_limits<double>::quiet_NaN())
  , gripper_a_cmd_atomic_(std::numeric_limits<double>::quiet_NaN())
  , gripper_a_pos_atomic_(std::numeric_limits<double>::quiet_NaN())
  , gripper_b_cmd_atomic_(std::numeric_limits<double>::quiet_NaN())
  , gripper_b_pos_atomic_(std::numeric_limits<double>::quiet_NaN())
  , gripper_thread_running_(false)
{
  RCLCPP_INFO(LOGGER, "Setting severity threshold to INFO");
  auto ret = rcutils_logging_set_logger_level(LOGGER.get_name(), RCUTILS_LOG_SEVERITY_INFO);
  if (ret != RCUTILS_RET_OK)
  {
    RCLCPP_ERROR(LOGGER, "Error setting severity: %s", rcutils_get_error_string().str);
    rcutils_reset_error();
  }
}

hardware_interface::CallbackReturn Kortex3HardwareInterfaceLowLevel::on_init(const hardware_interface::HardwareInfo& info)
{
  RCLCPP_INFO(LOGGER, "Initializing Kortex3 Hardware Interface...");
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  info_ = info;
  // The robot's IP address.
  robot_ip_ = info_.hardware_parameters["robot_ip"];
  if (robot_ip_.empty())
  {
    RCLCPP_ERROR(LOGGER, "Robot ip is empty!");
    return CallbackReturn::ERROR;
  }
  else
  {
    RCLCPP_INFO(LOGGER, "Robot ip is '%s'", robot_ip_.c_str());
  }
  // Username to log into the robot controller
  username_ = info_.hardware_parameters["username"];
  if (username_.empty())
  {
    RCLCPP_ERROR(LOGGER, "Username is empty!");
    return CallbackReturn::ERROR;
  }
  else
  {
    RCLCPP_INFO(LOGGER, "Username is '%s'", username_.c_str());
  }
  // Password to log into the robot controller
  password_ = info_.hardware_parameters["password"];
  if (password_.empty())
  {
    RCLCPP_ERROR(LOGGER, "Password is empty!");
    return CallbackReturn::ERROR;
  }
  // Port number for MQTT communication
  mqtt_port_ = std::stoi(info_.hardware_parameters["port"]);
  if (mqtt_port_ <= 0)
  {
    RCLCPP_ERROR(LOGGER, "Incorrect MQTT port number!");
    return CallbackReturn::ERROR;
  }
  else
  {
    RCLCPP_INFO(LOGGER, "MQTT port used '%d'", mqtt_port_);
  }
  // Port number for realtime (UDP) communication
  port_realtime_ = std::stoi(info_.hardware_parameters["port_realtime"]);
  if (port_realtime_ <= 0)
  {
    RCLCPP_ERROR(LOGGER, "Incorrect realtime port number!");
    return CallbackReturn::ERROR;
  }
  else
  {
    RCLCPP_INFO(LOGGER, "Realtime port used '%d'", port_realtime_);
  }
  // Inactivity period (in milliseconds) allowed before the session times out and closes on its own
  session_inactivity_timeout_ = std::stoi(info_.hardware_parameters["session_inactivity_timeout_ms"]);
  if (session_inactivity_timeout_ <= 0)
  {
    RCLCPP_ERROR(LOGGER, "Incorrect session inactivity timeout!");
    return CallbackReturn::ERROR;
  }
  else
  {
    RCLCPP_INFO(LOGGER, "Session inactivity timeout is '%d'", session_inactivity_timeout_);
  }
  // Inactivity period (in milliseconds) allowed before the robot stops any movements initiated from this session
  connection_inactivity_timeout_ = std::stoi(info_.hardware_parameters["connection_inactivity_timeout_ms"]);
  if (connection_inactivity_timeout_ <= 0)
  {
    RCLCPP_ERROR(LOGGER, "Incorrect connection inactivity timeout!");
    return CallbackReturn::ERROR;
  }
  else
  {
    RCLCPP_INFO(LOGGER, "Connection inactivity timeout is '%d'", connection_inactivity_timeout_);
  }

  // Safety system mode for the low-level position controller (optional, default: reduced)
  if (info_.hardware_parameters.count("safety_mode"))
  {
    const auto & v = info_.hardware_parameters.at("safety_mode");
    if (v == "normal")
    {
      low_level_safety_mode_ = k_api::SafetyFunctions::SAFETY_SYSTEM_MODE_NORMAL;
      RCLCPP_INFO(LOGGER, "Low-level safety mode: NORMAL");
    }
    else
    {
      low_level_safety_mode_ = k_api::SafetyFunctions::SAFETY_SYSTEM_MODE_REDUCED;
      RCLCPP_INFO(LOGGER, "Low-level safety mode: REDUCED");
    }
  }

  // Load gripper parameters (all optional)
  if (info_.hardware_parameters.count("use_internal_bus_gripper_comm"))
  {
    const auto & v = info_.hardware_parameters.at("use_internal_bus_gripper_comm");
    use_internal_bus_gripper_comm_ = (v == "true" || v == "True");
    if (use_internal_bus_gripper_comm_)
      RCLCPP_INFO(LOGGER, "Using internal bus communication for grippers.");
  }
  if (use_internal_bus_gripper_comm_)
  {
    if (info_.hardware_parameters.count("gripper_joint_name"))
      gripper_a_.joint_name_ = info_.hardware_parameters.at("gripper_joint_name");
    if (info_.hardware_parameters.count("gripper_modbus_id"))
      gripper_a_.modbus_id_ = static_cast<uint16_t>(
        std::stoul(info_.hardware_parameters.at("gripper_modbus_id")));
    if (info_.hardware_parameters.count("gripper_b_joint_name"))
      gripper_b_.joint_name_ = info_.hardware_parameters.at("gripper_b_joint_name");
    if (info_.hardware_parameters.count("gripper_b_modbus_id"))
      gripper_b_.modbus_id_ = static_cast<uint16_t>(
        std::stoul(info_.hardware_parameters.at("gripper_b_modbus_id")));
    // Closed-angle limit (URDF <limit upper>) is optional; defaults to 0.8 (2f_85). Use 0.7 for 2f_140.
    if (info_.hardware_parameters.count("gripper_max_angle"))
      gripper_a_.max_angle_ = std::stod(info_.hardware_parameters.at("gripper_max_angle"));
    if (info_.hardware_parameters.count("gripper_b_max_angle"))
      gripper_b_.max_angle_ = std::stod(info_.hardware_parameters.at("gripper_b_max_angle"));
    if (!gripper_a_.joint_name_.empty())
      RCLCPP_INFO(LOGGER, "Gripper A: joint='%s' modbus_id=%u",
                  gripper_a_.joint_name_.c_str(), gripper_a_.modbus_id_);
    if (!gripper_b_.joint_name_.empty())
      RCLCPP_INFO(LOGGER, "Gripper B: joint='%s' modbus_id=%u",
                  gripper_b_.joint_name_.c_str(), gripper_b_.modbus_id_);
  }

  // Check if expected command interfaces are present
  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    if (!(joint.command_interfaces[0].name == hardware_interface::HW_IF_POSITION ||
          joint.command_interfaces[0].name == hardware_interface::HW_IF_VELOCITY))
    {
      RCLCPP_FATAL(
        LOGGER, "Joint '%s' has %s command interface. Expected %s, or %s.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION,
        hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (!(joint.state_interfaces[0].name == hardware_interface::HW_IF_POSITION ||
          joint.state_interfaces[0].name == hardware_interface::HW_IF_VELOCITY ||
          joint.state_interfaces[0].name == hardware_interface::HW_IF_EFFORT))
    {
      RCLCPP_FATAL(
        LOGGER, "Joint '%s' has %s state interface. Expected %s, %s, or %s.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION,
        hardware_interface::HW_IF_VELOCITY, hardware_interface::HW_IF_EFFORT);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // Initialize state and command vectors.
  // Gripper joints are handled separately; only count arm actuators.
  actuator_count_ = 0;
  for (const auto & joint : info_.joints)
  {
    if (joint.name != gripper_a_.joint_name_ && joint.name != gripper_b_.joint_name_)
      actuator_count_++;
  }

  // The command interfaces need to be set to Nan if the joint trajectory controller is operating in open loop
  // See: https://control.ros.org/humble/doc/ros2_controllers/joint_trajectory_controller/doc/parameters.html
  joint_positions_.resize(actuator_count_, std::numeric_limits<double>::quiet_NaN());
  joint_velocities_.resize(actuator_count_, std::numeric_limits<double>::quiet_NaN());
  joint_torques_.resize(actuator_count_, std::numeric_limits<double>::quiet_NaN());
  joint_positions_cmd_.resize(actuator_count_, std::numeric_limits<double>::quiet_NaN());
  joint_velocities_cmd_.resize(actuator_count_, std::numeric_limits<double>::quiet_NaN());

  // set size of the twist interface
  twist_cmd_.resize(6, 0.0);

  // initialize kortex api twist commandd
  {
    k_api_twist_command_.set_reference_frame(k_api::Common::CARTESIAN_REFERENCE_FRAME_TOOL);
    k_api_twist_command_.set_duration(0);
    k_api_twist_ = k_api_twist_command_.mutable_twist();
  }

  RCLCPP_INFO(LOGGER, "Hardware Interface successfully initialized.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> Kortex3HardwareInterfaceLowLevel::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  std::vector<std::string> arm_joint_names;

  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    RCLCPP_DEBUG(LOGGER, "export_state_interfaces for joint: %s", info_.joints[i].name.c_str());
    if (!gripper_a_.joint_name_.empty() && info_.joints[i].name == gripper_a_.joint_name_)
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &gripper_a_pos_));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &gripper_a_vel_));
    }
    else if (!gripper_b_.joint_name_.empty() && info_.joints[i].name == gripper_b_.joint_name_)
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &gripper_b_pos_));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &gripper_b_vel_));
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

std::vector<hardware_interface::CommandInterface> Kortex3HardwareInterfaceLowLevel::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  std::vector<std::string> arm_joint_names;

  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    if (!gripper_a_.joint_name_.empty() && info_.joints[i].name == gripper_a_.joint_name_)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &gripper_a_cmd_));
    }
    else if (!gripper_b_.joint_name_.empty() && info_.joints[i].name == gripper_b_.joint_name_)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &gripper_b_cmd_));
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
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        arm_joint_names[i], hardware_interface::HW_IF_POSITION, &joint_positions_cmd_[i]));
  }

  // register twist command interfaces
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface("tcp", "twist.linear.x", &twist_cmd_[0]));
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface("tcp", "twist.linear.y", &twist_cmd_[1]));
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface("tcp", "twist.linear.z", &twist_cmd_[2]));
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface("tcp", "twist.angular.x", &twist_cmd_[3]));
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface("tcp", "twist.angular.y", &twist_cmd_[4]));
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface("tcp", "twist.angular.z", &twist_cmd_[5]));

  return command_interfaces;
}

hardware_interface::return_type Kortex3HardwareInterfaceLowLevel::prepare_command_mode_switch(
    const std::vector<std::string>& start_interfaces, const std::vector<std::string>& stop_interfaces)
{
  hardware_interface::return_type ret_val = hardware_interface::return_type::OK;

  // reset auxiliary switching booleans
  stop_low_level_control_mode_ = stop_twist_control_mode_ = stop_joint_velocity_control_mode_ = false;
  start_low_level_control_mode_ = start_twist_control_mode_ = start_joint_velocity_control_mode_ = false;

  // sleep to ensure all outgoing write commands have finished
  block_write_ = true;
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  start_modes_.clear();
  stop_modes_.clear();

  // Stopping interfaces
  // add stop interface per joint in tmp var for later check
  for (const auto& key : stop_interfaces)
  {
    for (auto& joint : info_.joints)
    {
      // Gripper joints are managed by the background thread; skip them here.
      if (joint.name == gripper_a_.joint_name_ || joint.name == gripper_b_.joint_name_)
        continue;

      if (key == joint.name + "/" + hardware_interface::HW_IF_POSITION)
      {
        stop_modes_.emplace_back(StopStartInterface::STOP_POS);
      }
      if (key == joint.name + "/" + hardware_interface::HW_IF_VELOCITY)
      {
        stop_modes_.emplace_back(StopStartInterface::STOP_VEL);
      }
      if (key == joint.name + "/" + hardware_interface::HW_IF_EFFORT)
      {
        // Effort command interface is not supported
        RCLCPP_ERROR(LOGGER, "Kortex3HardwareInterfaceLowLevel does not support effort command interface!");
        continue;
      }
    }
    if ((key == "tcp/twist.linear.x") || (key == "tcp/twist.linear.y") || (key == "tcp/twist.linear.z") ||
        (key == "tcp/twist.angular.x") || (key == "tcp/twist.angular.y") || (key == "tcp/twist.angular.z"))
    {
      stop_modes_.emplace_back(StopStartInterface::STOP_TWIST);
    }
  }

  // Starting interfaces
  for (const auto& key : start_interfaces)
  {
    for (auto& joint : info_.joints)
    {
      // Gripper joints are managed by the background thread; skip them here.
      if (joint.name == gripper_a_.joint_name_ || joint.name == gripper_b_.joint_name_)
        continue;

      if (key == joint.name + "/" + hardware_interface::HW_IF_POSITION)
      {
        start_modes_.emplace_back(StopStartInterface::START_POS);
      }
      if (key == joint.name + "/" + hardware_interface::HW_IF_VELOCITY)
      {
        start_modes_.emplace_back(StopStartInterface::START_VEL);
      }
      if (key == joint.name + "/" + hardware_interface::HW_IF_EFFORT)
      {
        // Effort command interface is not supported
        RCLCPP_ERROR(LOGGER, "Kortex3HardwareInterfaceLowLevel does not support effort command interface!");
        continue;
      }
    }
    if ((key == "tcp/twist.linear.x") || (key == "tcp/twist.linear.y") || (key == "tcp/twist.linear.z") ||
        (key == "tcp/twist.angular.x") || (key == "tcp/twist.angular.y") || (key == "tcp/twist.angular.z"))
    {
      start_modes_.emplace_back(StopStartInterface::START_TWIST);
    }
  }

  // prepare flags for performing the switch
  // Guard each stop flag with the currently running mode to avoid spurious stops when a controller
  // releases interfaces that belong to a different mode (e.g. JTC releasing velocity interfaces
  // while running in low-level position mode).
  if (!stop_modes_.empty() && low_level_control_mode_running_ &&
      std::find(stop_modes_.begin(), stop_modes_.end(), StopStartInterface::STOP_POS) != stop_modes_.end())
  {
    stop_low_level_control_mode_ = true;
  }
  if (!stop_modes_.empty() && joint_velocity_control_mode_running_ &&
      std::find(stop_modes_.begin(), stop_modes_.end(), StopStartInterface::STOP_VEL) != stop_modes_.end())
  {
    stop_joint_velocity_control_mode_ = true;
  }
  if (!stop_modes_.empty() && twist_control_mode_running_ &&
      std::find(stop_modes_.begin(), stop_modes_.end(), StopStartInterface::STOP_TWIST) != stop_modes_.end())
  {
    stop_twist_control_mode_ = true;
  }

  if (!start_modes_.empty() &&
      (std::find(start_modes_.begin(), start_modes_.end(), StopStartInterface::START_POS) != start_modes_.end()))
  {
    start_low_level_control_mode_ = true;
  }
  if (!start_modes_.empty() && !start_low_level_control_mode_ &&
      (std::find(start_modes_.begin(), start_modes_.end(), StopStartInterface::START_VEL) != start_modes_.end()))
  {
    start_joint_velocity_control_mode_ = true;
  }
  if (!start_modes_.empty() &&
      std::find(start_modes_.begin(), start_modes_.end(), StopStartInterface::START_TWIST) != start_modes_.end())
  {
    start_twist_control_mode_ = true;
  }

  // Handle exclusiveness between low_level, joint velocity, and twist control modes
  if ((start_low_level_control_mode_ && start_joint_velocity_control_mode_) ||
      (start_low_level_control_mode_ && start_twist_control_mode_) ||
      (start_joint_velocity_control_mode_ && start_twist_control_mode_))
  {
    RCLCPP_ERROR(LOGGER, "Starting multiple command interfaces at the same time is not supported!");
    return hardware_interface::return_type::ERROR;
  }
  if (low_level_control_mode_running_ && (start_twist_control_mode_ || start_joint_velocity_control_mode_) &&
      !stop_low_level_control_mode_)
  {
    RCLCPP_ERROR(LOGGER, "Can't start another control mode without disabling low-level control mode!");
    return hardware_interface::return_type::ERROR;
  }
  if (joint_velocity_control_mode_running_ && (start_twist_control_mode_ || start_low_level_control_mode_) &&
      !stop_joint_velocity_control_mode_)
  {
    RCLCPP_ERROR(LOGGER, "Can't start another control mode without disabling joint velocity control mode!");
    return hardware_interface::return_type::ERROR;
  }
  if (twist_control_mode_running_ && (start_low_level_control_mode_ || start_joint_velocity_control_mode_) &&
      !stop_twist_control_mode_)
  {
    RCLCPP_ERROR(LOGGER, "Can't start another control mode without disabling twist control mode!");
    return hardware_interface::return_type::ERROR;
  }

  return ret_val;
}

hardware_interface::return_type Kortex3HardwareInterfaceLowLevel::perform_command_mode_switch(
    const std::vector<std::string>& /*start_interfaces*/, const std::vector<std::string>& /*stop_interfaces*/)
{
  hardware_interface::return_type ret_val = hardware_interface::return_type::OK;

  // If low-level mode is being stopped and immediately restarted (e.g. switching from JTC to
  // cartesian controller), skip the hardware transition entirely — just sync the command buffers.
  if (stop_low_level_control_mode_ && start_low_level_control_mode_)
  {
    joint_positions_cmd_ = joint_positions_;
    joint_velocities_cmd_ = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    feedback_ = base_cyclic_->RefreshFeedback();
    stop_low_level_control_mode_ = false;
    start_low_level_control_mode_ = false;
  }

  if (stop_low_level_control_mode_)
  {
    set_servoing_mode(k_api::Base::SINGLE_LEVEL_SERVOING);
    change_operating_mode(k_api::Common::OPERATING_MODE_MONITORED_STOP);
    low_level_control_mode_running_ = false;
    joint_positions_cmd_ = joint_positions_;
  }
  if (stop_joint_velocity_control_mode_)
  {
    change_operating_mode(k_api::Common::OPERATING_MODE_MONITORED_STOP);
    joint_velocity_control_mode_running_ = false;
    joint_velocities_cmd_ = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  }
  if (stop_twist_control_mode_)
  {
    change_operating_mode(k_api::Common::OPERATING_MODE_MONITORED_STOP);
    twist_control_mode_running_ = false;
    twist_cmd_ = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  }

  if (start_low_level_control_mode_)
  {
    change_operating_mode(low_level_operating_mode_);
    set_safety_system_mode(low_level_safety_mode_);
    set_servoing_mode(k_api::Base::LOW_LEVEL_SERVOING);
    joint_velocity_control_mode_running_ = false;
    twist_control_mode_running_ = false;
    joint_positions_cmd_ = joint_positions_;
    joint_velocities_cmd_ = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    low_level_control_mode_running_ = true;
    feedback_ = base_cyclic_->RefreshFeedback();
  }
  if (start_joint_velocity_control_mode_)
  {
    set_servoing_mode(k_api::Base::SINGLE_LEVEL_SERVOING);
    change_operating_mode(k_api::Common::OPERATING_MODE_JOG_MANUAL);
    low_level_control_mode_running_ = false;
    twist_control_mode_running_ = false;
    joint_velocities_cmd_ = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    joint_velocity_control_mode_running_ = true;
  }
  if (start_twist_control_mode_)
  {
    set_servoing_mode(k_api::Base::SINGLE_LEVEL_SERVOING);
    change_operating_mode(k_api::Common::OPERATING_MODE_JOG_MANUAL);
    low_level_control_mode_running_ = false;
    joint_velocity_control_mode_running_ = false;
    twist_cmd_ = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    twist_control_mode_running_ = true;
  }

  // reset auxiliary switching booleans
  stop_low_level_control_mode_ = stop_twist_control_mode_ = stop_joint_velocity_control_mode_ = false;
  start_low_level_control_mode_ = start_twist_control_mode_ = start_joint_velocity_control_mode_ = false;

  start_modes_.clear();
  stop_modes_.clear();

  block_write_ = false;

  return ret_val;
}

hardware_interface::CallbackReturn
Kortex3HardwareInterfaceLowLevel::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_INFO(LOGGER, "Activating Kortex3 Hardware Interface...");

  try
  {
    // Initialize the Kortex API connection objects

    // MQTT: high-level, low-frequency commands
    router_mqtt_ = std::make_shared<k_api::RouterMQTT>(robot_ip_, mqtt_port_);
    router_mqtt_->SpinProcess(std::chrono::milliseconds{ 1 });
    session_mqtt_ = std::make_shared<k_api::Session::SessionClient>(router_mqtt_.get());
    base_mqtt_ = std::make_shared<k_api::Base::BaseClient>(router_mqtt_.get());
    safety_functions_client_ = std::make_shared<k_api::SafetyFunctions::SafetyFunctionsClient>(router_mqtt_.get());

    // UDP: high-frequency feedback and low-level commands
    transport_udp_ = std::make_unique<k_api::TransportClientUdp>();
    router_udp_ = std::make_unique<k_api::RouterClient>(transport_udp_.get(), [](k_api::KError err) {
      RCLCPP_ERROR(LOGGER, "UDP Router error: %s", err.toString().c_str());
    });
    session_udp_ = std::make_unique<k_api::SessionManager>(router_udp_.get());
    base_cyclic_ = std::make_shared<k_api::BaseCyclic::BaseCyclicClient>(router_udp_.get());

    // Start UDP connection
    transport_udp_->connect(robot_ip_, port_realtime_);

    // Set session data connection information
    auto session_info = k_api::Session::CreateSessionInfo();
    session_info.set_username(username_);
    session_info.set_password(password_);
    session_info.set_session_inactivity_timeout(session_inactivity_timeout_);
    session_info.set_connection_inactivity_timeout(connection_inactivity_timeout_);

    // Session manager service wrapper
    RCLCPP_INFO(LOGGER, "Creating session for communication");
    session_mqtt_->CreateSession(session_info);
    session_udp_->CreateSession(session_info);
    RCLCPP_INFO(LOGGER, "Session created");

    // Wait for session to establish
    std::this_thread::sleep_for(std::chrono::milliseconds(150));

    // Get the actual actuator count from the robot and verify if it matches the URDF
    auto actuator_info = base_mqtt_->GetActuatorCount();
    if (actuator_info.count() != actuator_count_)
    {
      RCLCPP_ERROR(LOGGER, "Robot reports %d actuators, but URDF expected %zu.",
        actuator_info.count(), actuator_count_);
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Set single level servoing and Monitored Stop on startup
    set_servoing_mode(k_api::Base::SINGLE_LEVEL_SERVOING);
    change_operating_mode(k_api::Common::OPERATING_MODE_MONITORED_STOP);

    // First read
    auto base_feedback = base_cyclic_->RefreshFeedback();

    // Set some default values
    for (std::size_t i = 0; i < actuator_count_; i++)
    {
      if (std::isnan(joint_positions_[i]))
      {
        joint_positions_[i] = base_feedback.actuators(i).position() * M_PI / 180.0;  // rad
      }
      if (std::isnan(joint_velocities_[i]))
      {
        joint_velocities_[i] = 0;
      }
      if (std::isnan(joint_torques_[i]))
      {
        joint_torques_[i] = 0;
      }
      if (std::isnan(joint_positions_cmd_[i]))
      {
        joint_positions_cmd_[i] = base_feedback.actuators(i).position() * M_PI / 180.0;  // rad
      }
      if (std::isnan(joint_velocities_cmd_[i]))
      {
        joint_velocities_cmd_[i] = 0;
      }
    }

    // Initialize grippers over the shared MQTT session
    if (use_internal_bus_gripper_comm_)
    {
      if (!gripper_a_.joint_name_.empty())
      {
        if (!gripper_a_.initialize(router_mqtt_, LOGGER))
        {
          RCLCPP_ERROR(LOGGER, "Failed to initialize Gripper A.");
          return hardware_interface::CallbackReturn::ERROR;
        }
        auto pos = gripper_a_.readPosition(gripper_mtx_, LOGGER);
        double init = pos.has_value() ? pos.value() : 0.0;
        gripper_a_pos_ = init;
        gripper_a_cmd_ = init;
        gripper_a_pos_atomic_.store(init, std::memory_order_relaxed);
        gripper_a_cmd_atomic_.store(init, std::memory_order_relaxed);
        RCLCPP_INFO(LOGGER, "Gripper A initial position: %.4f rad", init);
      }
      if (!gripper_b_.joint_name_.empty())
      {
        if (!gripper_b_.initialize(router_mqtt_, LOGGER))
        {
          RCLCPP_ERROR(LOGGER, "Failed to initialize Gripper B.");
          return hardware_interface::CallbackReturn::ERROR;
        }
        auto pos = gripper_b_.readPosition(gripper_mtx_, LOGGER);
        double init = pos.has_value() ? pos.value() : 0.0;
        gripper_b_pos_ = init;
        gripper_b_cmd_ = init;
        gripper_b_pos_atomic_.store(init, std::memory_order_relaxed);
        gripper_b_cmd_atomic_.store(init, std::memory_order_relaxed);
        RCLCPP_INFO(LOGGER, "Gripper B initial position: %.4f rad", init);
      }

      // Launch the background thread that handles all Modbus I/O at ~20 Hz.
      // The CM update loop only touches the atomic buffers (nanosecond operations).
      gripper_thread_running_.store(true, std::memory_order_relaxed);
      gripper_thread_ = std::thread(&Kortex3HardwareInterfaceLowLevel::gripperThreadLoop, this);
      RCLCPP_INFO(LOGGER, "Gripper background thread started.");
    }

    RCLCPP_INFO(LOGGER, "Kortex3 Hardware Interface successfully activated.");
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  catch (const std::exception& ex)
  {
    RCLCPP_ERROR(LOGGER, "Exception during activation: %s", ex.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
Kortex3HardwareInterfaceLowLevel::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_INFO(LOGGER, "Deactivating Kortex Hardware Interface...");

  // Stop the gripper background thread before closing the MQTT session it uses.
  if (gripper_thread_running_.load(std::memory_order_relaxed))
  {
    gripper_thread_running_.store(false, std::memory_order_relaxed);
    if (gripper_thread_.joinable())
      gripper_thread_.join();
    RCLCPP_INFO(LOGGER, "Gripper background thread stopped.");
  }
  if (!gripper_a_.joint_name_.empty()) gripper_a_.shutdown(gripper_mtx_);
  if (!gripper_b_.joint_name_.empty()) gripper_b_.shutdown(gripper_mtx_);

  // Set back the servoing mode to Single Level Servoing and Operating mode to Monitored Stop
  set_servoing_mode(k_api::Base::SINGLE_LEVEL_SERVOING);
  change_operating_mode(k_api::Common::OPERATING_MODE_MONITORED_STOP);

  // Close API sessions
  if (session_udp_)
  {
    try
    {
      session_udp_->CloseSession();
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(LOGGER, "Error closing UDP session: %s", e.what());
    }
  }
  if (session_mqtt_)
  {
    try
    {
      session_mqtt_->CloseSession();
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(LOGGER, "Error closing MQTT session: %s", e.what());
    }
  }
  // Deactivate the router and cleanly disconnect from the transport object
  if (router_mqtt_)
  {
    router_mqtt_->SetActivationStatus(false);
  }
  if (router_udp_)
  {
    router_udp_->SetActivationStatus(false);
  }
  if (transport_udp_)
  {
    try
    {
      transport_udp_->disconnect();
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(LOGGER, "Error disconnecting UDP transport: %s", e.what());
    }
  }

  // Memory handling
  delete k_api_twist_;

  RCLCPP_INFO(LOGGER, "Kortex Hardware Interface deactivated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type Kortex3HardwareInterfaceLowLevel::read(const rclcpp::Time& /*time*/,
                                                              const rclcpp::Duration& /*period*/)
{

  // Read the arm state from the feedback
  arm_state_ = feedback_.base().active_state();
  operating_mode_ = feedback_.base().operating_mode();
  enabling_device_state_ = feedback_.base().enabling_device_state();

  if (operating_mode_ == k_api::Common::OPERATING_MODE_MONITORED_STOP)
  {
    RCLCPP_WARN_THROTTLE(LOGGER, clock_, 3000, "Robot is in Monitored Stop and will not move.");
  }

  if ((operating_mode_ == k_api::Common::OPERATING_MODE_HOLD_TO_RUN || 
       operating_mode_ == k_api::Common::OPERATING_MODE_JOG_MANUAL) &&
      !enabling_device_state_)
  {
    RCLCPP_WARN_THROTTLE(LOGGER, clock_, 3000,
                        "Robot will not move - the enabling device is not pressed. Hold the three-position enabling "
                        "device in its middle position to enable the controller.");
  }

  // Detect if the arm is in fault
  in_fault_ = (arm_state_ == k_api::Common::ArmState::ARMSTATE_IN_FAULT ||
      arm_state_ == k_api::Common::ArmState::ARMSTATE_IN_FAULT_POWERED_OFF);

  // If the enabling device is released in JOG_MANUAL mode, the arm swithces automatically
  // to MONITORED_STOP. We need to change it back to JOG_MANUAL once the enabling device is back
  if ((joint_velocity_control_mode_running_ || twist_control_mode_running_) &&
      operating_mode_ == k_api::Common::OPERATING_MODE_MONITORED_STOP && 
      enabling_device_state_ && !in_fault_)
  {
    change_operating_mode(k_api::Common::OPERATING_MODE_JOG_MANUAL);
  }

  try
  {
    for (size_t i = 0; i < feedback_.actuators_size() && i < actuator_count_; ++i)
    {
      const auto& act = feedback_.actuators(i);
      joint_positions_[i] = act.position() * M_PI / 180.0;
      joint_velocities_[i] = act.velocity() * M_PI / 180.0;
      joint_torques_[i] = act.torque();
    }
  }
  catch (const k_api::KDetailedException& ex)
  {
    RCLCPP_ERROR(LOGGER, "Robot feedback error: %s", ex.what());
    return hardware_interface::return_type::ERROR;
  }

  // Copy gripper positions from atomic buffers written by the background thread.
  // This is a nanosecond operation — no Modbus I/O on the CM thread.
  if (use_internal_bus_gripper_comm_)
  {
    if (!gripper_a_.joint_name_.empty())
      gripper_a_pos_ = gripper_a_pos_atomic_.load(std::memory_order_relaxed);
    if (!gripper_b_.joint_name_.empty())
      gripper_b_pos_ = gripper_b_pos_atomic_.load(std::memory_order_relaxed);
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type Kortex3HardwareInterfaceLowLevel::write(const rclcpp::Time& /*time*/,
                                                               const rclcpp::Duration& /*period*/)
{
  if (operating_mode_ == k_api::Common::OPERATING_MODE_MONITORED_STOP ||
      ((operating_mode_ == k_api::Common::OPERATING_MODE_HOLD_TO_RUN || 
       operating_mode_ == k_api::Common::OPERATING_MODE_JOG_MANUAL) &&
      !enabling_device_state_) || block_write_)
  {
    feedback_ = base_cyclic_->RefreshFeedback();
    return hardware_interface::return_type::OK;
  }

  if (!in_fault_)
  {
    if (arm_mode_ == k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING)
    {
      // High-level control modes
      if (joint_velocity_control_mode_running_)
      {
        sendJointSpeedsCommand();
      }
      else if (twist_control_mode_running_)
      {
        sendTwistCommand();
      }
      else
      {
        // Keep alive mode - no controller active
        RCLCPP_DEBUG(LOGGER, "No controller active in SINGLE_LEVEL_SERVOING mode!");
      }
      // Read after write in high-level mode
      feedback_ = base_cyclic_->RefreshFeedback();
    }
    else if (arm_mode_ == k_api::Base::ServoingMode::LOW_LEVEL_SERVOING)
    {
      // Low level control mode

      if (low_level_control_mode_running_)
      {
        sendJointPositionCommands();
      }
      else
      {
        // Keep alive mode - no controller active
        feedback_ = base_cyclic_->RefreshFeedback();
        RCLCPP_DEBUG(LOGGER, "No controller active in LOW_LEVEL_SERVOING mode !");
      }
    }
    else
    {
      // Keep alive mode - no controller active
      feedback_ = base_cyclic_->RefreshFeedback();
      RCLCPP_DEBUG(LOGGER,
                   "Fault was not recognized on the robot but combination of Control Mode and Active State "
                   "are not supported!");
    }
  }
  else
  {
    // this is needed when the robot was faulted
    // so we can internally conclude it is not faulted anymore
    feedback_ = base_cyclic_->RefreshFeedback();
    RCLCPP_WARN_THROTTLE(LOGGER, clock_, 3000, "Robot is in fault!");
  }

  // Forward gripper commands to the background thread via atomics.
  // This is a nanosecond operation — no Modbus I/O on the CM thread.
  if (use_internal_bus_gripper_comm_)
  {
    if (!gripper_a_.joint_name_.empty() && !std::isnan(gripper_a_cmd_))
      gripper_a_cmd_atomic_.store(gripper_a_cmd_, std::memory_order_relaxed);
    if (!gripper_b_.joint_name_.empty() && !std::isnan(gripper_b_cmd_))
      gripper_b_cmd_atomic_.store(gripper_b_cmd_, std::memory_order_relaxed);
  }

  return hardware_interface::return_type::OK;
}

void Kortex3HardwareInterfaceLowLevel::change_operating_mode(const k_api::Common::OperatingModeType& mode)
{
  // The possible operating mode types of the robots are:

  // OPERATING_MODE_UNSPECIFIED (0):       Unspecified operating mode
  // OPERATING_MODE_JOG_MANUAL (1):        Jog manual operating mode
  // OPERATING_MODE_HAND_GUIDING (2):      Hand guiding operating mode
  // OPERATING_MODE_HOLD_TO_RUN (3):       Hold to run operating mode
  // OPERATING_MODE_AUTO (4):              Automatic operating mode
  // OPERATING_MODE_MONITORED_STOP (5):    Monitored stop operating mode

  try
  {
    mode_selection_.set_operating_mode(mode);
    base_mqtt_->SelectOperatingMode(mode_selection_);
    // Allow time for the controller to switch modes.
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    RCLCPP_INFO(LOGGER, "Operating mode set to %s.", k_api::Common::OperatingModeType_Name(mode).c_str());
  }
  catch (const k_api::KDetailedException& ex)
  {
    RCLCPP_ERROR(LOGGER, "Failed to change operating mode: %s", ex.what());
  }
  catch (std::runtime_error& ex_runtime)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Runtime error: " << ex_runtime.what());
  }
  catch (std::future_error& ex_future)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Future error: " << ex_future.what());
  }
  catch (std::exception& ex_std)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Standard exception: " << ex_std.what());
  }
}

void Kortex3HardwareInterfaceLowLevel::set_servoing_mode(const k_api::Base::ServoingMode& mode)
{
  // The possible servoing modes of the robots are:

  // UNSPECIFIED_SERVOING_MODE (0):    Unspecified servoing mode
  // SINGLE_LEVEL_SERVOING (2):        Single level servoing mode (high-level)
  // LOW_LEVEL_SERVOING (3):           Low level servoing mode
  // BYPASS_SERVOING (4):              Bypass servoing mode

  try
  {
    servoing_mode_info_.set_servoing_mode(mode);
    base_mqtt_->SetServoingMode(servoing_mode_info_);
    arm_mode_ = mode;
    RCLCPP_INFO(LOGGER, "Servoing mode set to %s.", k_api::Base::ServoingMode_Name(mode).c_str());
  }
  catch (const k_api::KDetailedException& ex)
  {
    RCLCPP_ERROR(LOGGER, "Failed to set servoing mode: %s", ex.what());
  }
  catch (std::runtime_error& ex_runtime)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Runtime error: " << ex_runtime.what());
  }
  catch (std::future_error& ex_future)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Future error: " << ex_future.what());
  }
  catch (std::exception& ex_std)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Standard exception: " << ex_std.what());
  }
}

void Kortex3HardwareInterfaceLowLevel::set_safety_system_mode(const k_api::SafetyFunctions::SafetySystemMode& mode)
{
  // The possible safety system modes are:

  // SAFETY_SYSTEM_MODE_UNSPECIFIED (0):    Unspecified safety system mode
  // SAFETY_SYSTEM_MODE_NORMAL (1):         Normal safety system mode (faster joint limits)
  // SAFETY_SYSTEM_MODE_REDUCED (2):        Reduced safety system mode (slower joint limits)

  try
  {
    safety_system_.set_mode(mode);
    safety_functions_client_->SetSafetySystemMode(safety_system_);
    RCLCPP_INFO(LOGGER, "Setting safety system mode to %s.", k_api::SafetyFunctions::SafetySystemMode_Name(mode).c_str());
  }
  catch (const k_api::KDetailedException& ex)
  {
    const bool is_already_set =
        ex.getErrorInfo().getError().error_sub_code() == k_api::SubErrorCodes::INVALID_PARAM &&
        std::string(ex.what()).find("same as the current safety mode") != std::string::npos;

    if (is_already_set)
    {
      RCLCPP_INFO(LOGGER, "Safety system mode already set to %s.",
                   k_api::SafetyFunctions::SafetySystemMode_Name(mode).c_str());
    }
    else
    {
      RCLCPP_ERROR(LOGGER, "Failed to set safety system mode: %s", ex.what());
    }
  }
  catch (std::runtime_error& ex_runtime)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Runtime error: " << ex_runtime.what());
  }
  catch (std::future_error& ex_future)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Future error: " << ex_future.what());
  }
  catch (std::exception& ex_std)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Standard exception: " << ex_std.what());
  }
}

void Kortex3HardwareInterfaceLowLevel::sendJointSpeedsCommand()
{
  try
  {
    k_api::Base::JointSpeeds joint_speeds;
    for (size_t i = 0; i < actuator_count_; ++i)
    {
      auto joint_speed = joint_speeds.add_joint_speeds();
      joint_speed->set_joint_identifier(i);
      joint_speed->set_value(static_cast<float>(joint_velocities_cmd_[i] * 180.0 / M_PI));
    }
    base_mqtt_->SendJointSpeedsCommand(joint_speeds);
  }
  catch (const k_api::KDetailedException& e)
  {
    RCLCPP_ERROR(LOGGER, "Unexpected fault during write(): %s", e.what());
    RCLCPP_ERROR_STREAM(LOGGER, "Error sub-code: " << k_api::SubErrorCodes_Name(
                                    k_api::SubErrorCodes((e.getErrorInfo().getError().error_sub_code()))));
  }
}

void Kortex3HardwareInterfaceLowLevel::sendJointPositionCommands()
{
  base_command_frame_id_ = base_command_frame_id_ + 1;
  if (base_command_frame_id_ > 65535)
    base_command_frame_id_ = 0;

  k_api::BaseCyclic::Command command;
  command.set_frame_id(base_command_frame_id_);
  for (size_t i = 0; i < actuator_count_; i++)
  {
    auto* actuator = command.add_actuators();
    actuator->set_flags(0);
    actuator->set_torque_joint(0.0f);  // Force PID to accumulate gravity comp
    actuator->set_position(joint_positions_cmd_[i] * 180.0 / M_PI);
    actuator->set_velocity(joint_velocities_cmd_[i] * 180.0 / M_PI);
  }

  // send the command to the robot
  try
  {
    feedback_ = base_cyclic_->Refresh(command);
  }
  catch (k_api::KDetailedException& ex)
  {
    feedback_ = base_cyclic_->RefreshFeedback();
    RCLCPP_ERROR_STREAM(LOGGER, "Kortex exception: " << ex.what());

    RCLCPP_ERROR_STREAM(LOGGER, "Error sub-code: " << k_api::SubErrorCodes_Name(
                                    k_api::SubErrorCodes((ex.getErrorInfo().getError().error_sub_code()))));
  }
  catch (std::runtime_error& ex_runtime)
  {
    feedback_ = base_cyclic_->RefreshFeedback();
    RCLCPP_ERROR_STREAM(LOGGER, "Runtime error: " << ex_runtime.what());
  }
  catch (std::future_error& ex_future)
  {
    feedback_ = base_cyclic_->RefreshFeedback();
    RCLCPP_ERROR_STREAM(LOGGER, "Future error: " << ex_future.what());
  }
  catch (std::exception& ex_std)
  {
    feedback_ = base_cyclic_->RefreshFeedback();
    RCLCPP_ERROR_STREAM(LOGGER, "Standard exception: " << ex_std.what());
  }
}

void Kortex3HardwareInterfaceLowLevel::sendTwistCommand()
{
  try
  {
    k_api_twist_->set_linear_x(twist_cmd_[0]);
    k_api_twist_->set_linear_y(twist_cmd_[1]);
    k_api_twist_->set_linear_z(twist_cmd_[2]);
    k_api_twist_->set_angular_x(twist_cmd_[3] * 180.0 / M_PI);
    k_api_twist_->set_angular_y(twist_cmd_[4] * 180.0 / M_PI);
    k_api_twist_->set_angular_z(twist_cmd_[5] * 180.0 / M_PI);
    base_mqtt_->SendTwistCommand(k_api_twist_command_);
  }
  catch (const k_api::KDetailedException& e)
  {
    RCLCPP_ERROR(LOGGER, "Unexpected error when sending twist command: %s", e.what());
    RCLCPP_ERROR_STREAM(LOGGER, "Error sub-code: " << k_api::SubErrorCodes_Name(
                                    k_api::SubErrorCodes((e.getErrorInfo().getError().error_sub_code()))));
  }
}

void Kortex3HardwareInterfaceLowLevel::gripperThreadLoop()
{
  // Run at ~20 Hz. GripperController::sendCommand and readPosition have their own
  // internal rate limiting, so calling them every iteration is safe.
  constexpr auto period = std::chrono::milliseconds(50);
  auto next_wakeup = std::chrono::steady_clock::now() + period;

  while (gripper_thread_running_.load(std::memory_order_relaxed))
  {
    if (!gripper_a_.joint_name_.empty() && gripper_a_.initialized_ && !in_fault_)
    {
      // Read position from hardware and publish to atomic
      auto pos = gripper_a_.readPosition(gripper_mtx_, LOGGER);
      if (pos.has_value())
        gripper_a_pos_atomic_.store(pos.value(), std::memory_order_relaxed);

      // Fetch the latest command written by the CM thread and send to hardware
      const double cmd = gripper_a_cmd_atomic_.load(std::memory_order_relaxed);
      if (!std::isnan(cmd))
        gripper_a_.sendCommand(cmd, gripper_mtx_);
    }

    if (!gripper_b_.joint_name_.empty() && gripper_b_.initialized_ && !in_fault_)
    {
      auto pos = gripper_b_.readPosition(gripper_mtx_, LOGGER);
      if (pos.has_value())
        gripper_b_pos_atomic_.store(pos.value(), std::memory_order_relaxed);

      const double cmd = gripper_b_cmd_atomic_.load(std::memory_order_relaxed);
      if (!std::isnan(cmd))
        gripper_b_.sendCommand(cmd, gripper_mtx_);
    }

    std::this_thread::sleep_until(next_wakeup);
    next_wakeup += period;
  }
}

}  // namespace kortex3_driver

#include "pluginlib/class_list_macros.hpp"

// Registers this class with pluginlib, making it available to the ros2_control controller manager.
PLUGINLIB_EXPORT_CLASS(kortex3_driver::Kortex3HardwareInterfaceLowLevel, hardware_interface::SystemInterface)