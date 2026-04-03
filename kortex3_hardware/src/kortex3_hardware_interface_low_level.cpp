
#include <chrono>
#include <string>
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
  // TODO: Add a description for the parameter
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
  // TODO: Add a description for the parameer
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
  // TODO: Load gripper parameters
  // gripper joint name
  // gripper_joint_name_ = info_.hardware_parameters["gripper_joint_name"];
  // if (gripper_joint_name_.empty())
  // {
  //   RCLCPP_ERROR(LOGGER, "Gripper joint name is empty!");
  // }
  // else
  // {
  //   RCLCPP_INFO(LOGGER, "Gripper joint name is '%s'", gripper_joint_name_.c_str());
  // }

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

  // Initialize state and command vectors
  actuator_count_ = info_.joints.size();

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
    // command.set_duration = execute time (milliseconds) according to the api ->
    // (not implemented yet)
    // see: https://github.com/Kinovarobotics/kortex/blob/master/api_cpp/doc/markdown/messages/Base/TwistCommand.md
    k_api_twist_command_.set_duration(0);
    k_api_twist_ = k_api_twist_command_.mutable_twist();
  }

  // TODO: Check and report if using internal bus for gripper
  // if (
  //   (info_.hardware_parameters["use_internal_bus_gripper_comm"] == "true") ||
  //   (info_.hardware_parameters["use_internal_bus_gripper_comm"] == "True"))
  // {
  //   use_internal_bus_gripper_comm_ = true;
  //   RCLCPP_INFO(LOGGER, "Using internal bus communication for gripper!");
  // }

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
    // TODO: Export gripper interfaces
    // if (info_.joints[i].name == gripper_a_.joint_name_)
    // {
    //   state_interfaces.emplace_back(hardware_interface::StateInterface(
    //     info_.joints[i].name, hardware_interface::HW_IF_POSITION, &gripper_a_.position_));
    //   state_interfaces.emplace_back(hardware_interface::StateInterface(
    //     info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &gripper_a_.velocity_));
    // }
    // else if (info_.joints[i].name == gripper_b_.joint_name_)
    // {
    //   state_interfaces.emplace_back(hardware_interface::StateInterface(
    //     info_.joints[i].name, hardware_interface::HW_IF_POSITION, &gripper_b_.position_));
    //   state_interfaces.emplace_back(hardware_interface::StateInterface(
    //     info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &gripper_b_.velocity_));
    // }
    // else
    // {
    arm_joint_names.emplace_back(info_.joints[i].name);
    // }
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
    // TODO: Export gripper interfaces
    // if (info_.joints[i].name == gripper_a_.joint_name_)
    // {
    //   command_interfaces.emplace_back(hardware_interface::CommandInterface(
    //     info_.joints[i].name, hardware_interface::HW_IF_POSITION, &gripper_a_.command_position_));
    // }
    // else if (info_.joints[i].name == gripper_b_.joint_name_)
    // {
    //   command_interfaces.emplace_back(hardware_interface::CommandInterface(
    //     info_.joints[i].name, hardware_interface::HW_IF_POSITION, &gripper_b_.command_position_));
    // }
    // else
    // {
    arm_joint_names.emplace_back(info_.joints[i].name);
    // }
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
      // TODO: Include gripper joints
      // if (
      //   key == joint.name + "/" + hardware_interface::HW_IF_POSITION &&
      //   joint.name == gripper_joint_name_)
      // {
      //   stop_modes_.emplace_back(StopStartInterface::STOP_GRIPPER);
      //   continue;
      // }
      // if (
      //   key == joint.name + "/" + hardware_interface::HW_IF_VELOCITY &&
      //   joint.name == gripper_joint_name_)
      // {
      //   continue;
      // }
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
    // TODO: Include fault controller joints
    // if ((key == "reset_fault/command") || (key == "reset_fault/async_success"))
    // {
    //   stop_modes_.emplace_back(StopStartInterface::STOP_FAULT_CTRL);
    // }
  }

  // Starting interfaces
  // add start interface per joint in tmp var for later check
  for (const auto& key : start_interfaces)
  {
    for (auto& joint : info_.joints)
    {
      // TODO: Include gripper joints
      // if (
      //   key == joint.name + "/" + hardware_interface::HW_IF_POSITION &&
      //   joint.name == gripper_joint_name_)
      // {
      //   start_modes_.emplace_back(StopStartInterface::START_GRIPPER);
      //   continue;
      // }
      // if (
      //   key == joint.name + "/" + hardware_interface::HW_IF_VELOCITY &&
      //   joint.name == gripper_joint_name_)
      // {
      //   continue;
      // }
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
    // TODO: Include fault controller joints
    // if ((key == "reset_fault/command") || (key == "reset_fault/async_success"))
    // {
    //   start_modes_.emplace_back(StopStartInterface::START_FAULT_CTRL);
    // }
  }

  // prepare flags for performing the switch
  if (!stop_modes_.empty() &&
      std::find(stop_modes_.begin(), stop_modes_.end(), StopStartInterface::STOP_POS) != stop_modes_.end())
  {
    stop_low_level_control_mode_ = true;
  }
  if (!stop_modes_.empty() &&
      std::find(stop_modes_.begin(), stop_modes_.end(), StopStartInterface::STOP_VEL) != stop_modes_.end())
  {
    stop_joint_velocity_control_mode_ = true;
  }
  if (!stop_modes_.empty() &&
      std::find(stop_modes_.begin(), stop_modes_.end(), StopStartInterface::STOP_TWIST) != stop_modes_.end())
  {
    stop_twist_control_mode_ = true;
  }
  // TODO: Include the gripper and fault controllers
  // if (
  //   !stop_modes_.empty() &&
  //   std::find(stop_modes_.begin(), stop_modes_.end(), StopStartInterface::STOP_GRIPPER) !=
  //     stop_modes_.end())
  // {
  //   stop_gripper_controller_ = true;
  // }
  // if (
  //   !stop_modes_.empty() &&
  //   std::find(stop_modes_.begin(), stop_modes_.end(), StopStartInterface::STOP_FAULT_CTRL) !=
  //     stop_modes_.end())
  // {
  //   stop_fault_controller_ = true;
  // }

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
  // TODO: Include the gripper and fault controllers
  // if (
  //   !start_modes_.empty() &&
  //   (std::find(start_modes_.begin(), start_modes_.end(), StopStartInterface::START_GRIPPER) !=
  //    start_modes_.end()))
  // {
  //   start_gripper_controller_ = true;
  // }
  // if (
  //   !start_modes_.empty() &&
  //   (std::find(start_modes_.begin(), start_modes_.end(), StopStartInterface::START_FAULT_CTRL) !=
  //    start_modes_.end()))
  // {
  //   start_fault_controller_ = true;
  // }

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

  if (stop_low_level_control_mode_)
  {
    stop_low_level_mode();
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
    start_low_level_mode();
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
  // TODO: Include gripper and fault controllers
  // if (start_gripper_controller_)
  // {
  //   gripper_command_position_ = gripper_position_;
  //   gripper_controller_running_ = true;
  // }
  // if (start_fault_controller_)
  // {
  //   fault_controller_running_ = true;
  // }

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

  // Set back the servoing mode to Single Level Servoing and Operating mode to Monitored Stop
  if (low_level_control_mode_running_)
  {
    stop_low_level_mode();
  }
  else
  {
    change_operating_mode(k_api::Common::OPERATING_MODE_MONITORED_STOP);
  }

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

  // TODO: Shutdown grippers
  // if (!gripper_a_.joint_name_.empty())
  // {
  //   gripper_a_.shutdown(gripper_mtx_);
  // }
  // if (!gripper_b_.joint_name_.empty())
  // {
  //   gripper_b_.shutdown(gripper_mtx_);
  // }

  // Memory handling
  delete k_api_twist_;

  RCLCPP_INFO(LOGGER, "Kortex Hardware Interface deactivated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type Kortex3HardwareInterfaceLowLevel::read(const rclcpp::Time& /*time*/,
                                                              const rclcpp::Duration& /*period*/)
{
  try
  {
    feedback_ = base_cyclic_->RefreshFeedback();

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

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type Kortex3HardwareInterfaceLowLevel::write(const rclcpp::Time& /*time*/,
                                                               const rclcpp::Duration& /*period*/)
{
  if (block_write_)
  {
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

      // TODO: gripper control
      // sendGripperCommand(
      //   arm_mode_, gripper_command_position_, gripper_speed_command_, gripper_force_command_);
      // read after write in twist mode
      // feedback_ = base_cyclic_->RefreshFeedback();
    }
    else if (arm_mode_ == k_api::Base::ServoingMode::LOW_LEVEL_SERVOING)
    {
      // Low level control mode

      // TODO: gripper control
      // sendGripperCommand(
      //   arm_mode_, gripper_command_position_, gripper_speed_command_, gripper_force_command_);

      if (low_level_control_mode_running_)
      {
        sendJointPositionCommands();
      }
      else
      {
        // Keep alive mode - no controller active
        RCLCPP_DEBUG(LOGGER, "No controller active in LOW_LEVEL_SERVOING mode !");
      }
    }
    else
    {
      // Keep alive mode - no controller active
      RCLCPP_DEBUG(LOGGER,
                   "Fault was not recognized on the robot but combination of Control Mode and Active State "
                   "are not supported!");
    }
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

void Kortex3HardwareInterfaceLowLevel::start_low_level_mode()
{
  change_operating_mode(k_api::Common::OPERATING_MODE_MONITORED_STOP);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  change_operating_mode(k_api::Common::OPERATING_MODE_AUTO);
  set_servoing_mode(k_api::Base::LOW_LEVEL_SERVOING);
}

void Kortex3HardwareInterfaceLowLevel::stop_low_level_mode()
{
  set_servoing_mode(k_api::Base::SINGLE_LEVEL_SERVOING);
  std::this_thread::sleep_for(std::chrono::milliseconds(3500));
  change_operating_mode(k_api::Common::OPERATING_MODE_MONITORED_STOP);
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
  k_api_twist_->set_linear_x(twist_cmd_[0]);
  k_api_twist_->set_linear_y(twist_cmd_[1]);
  k_api_twist_->set_linear_z(twist_cmd_[2]);
  k_api_twist_->set_angular_x(twist_cmd_[3] * 180.0 / M_PI);
  k_api_twist_->set_angular_y(twist_cmd_[4] * 180.0 / M_PI);
  k_api_twist_->set_angular_z(twist_cmd_[5] * 180.0 / M_PI);
  base_mqtt_->SendTwistCommand(k_api_twist_command_);
}

}  // namespace kortex3_driver

#include "pluginlib/class_list_macros.hpp"

// Registers this class with pluginlib, making it available to the ros2_control controller manager.
PLUGINLIB_EXPORT_CLASS(kortex3_driver::Kortex3HardwareInterfaceLowLevel, hardware_interface::SystemInterface)