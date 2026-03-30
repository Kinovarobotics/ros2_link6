
#include <chrono>
#include <string>
#include <vector>

#include "kortex3_hardware/kortex_hardware_interface.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace kortex3_driver
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("KortexHardwareInterface");

KortexHardwareInterface::KortexHardwareInterface()
  : mode_selection_(k_api::Common::ModeSelection())
  , servoing_mode_info_(k_api::Base::ServoingModeInformation())
  , actuator_count_(6)  // Default, updated from robot during activation.
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
  RCLCPP_INFO(LOGGER, "Setting severity threshold to DEBUG");
  auto ret = rcutils_logging_set_logger_level(LOGGER.get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
  if (ret != RCUTILS_RET_OK)
  {
    RCLCPP_ERROR(LOGGER, "Error setting severity: %s", rcutils_get_error_string().str);
    rcutils_reset_error();
  }

  // Initialize the Kortex API connection objects

  // MQTT

  // UDP
  transport_udp_ = std::make_unique<k_api::TransportClientUdp>();
  router_udp_ = std::make_unique<k_api::RouterClient>(transport_udp_.get(), [](k_api::KError err) {
    RCLCPP_ERROR(LOGGER, "UDP Router error: %s", err.toString().c_str());
  });
  session_udp_ = std::make_unique<k_api::SessionManager>(router_udp_.get());

  base_cyclic_ = std::make_shared<k_api::BaseCyclic::BaseCyclicClient>(router_udp_.get());
}

hardware_interface::CallbackReturn KortexHardwareInterface::on_init(const hardware_interface::HardwareInfo& info)
{
  RCLCPP_INFO(LOGGER, "Initializing Kortex3 Hardware Interface...");
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  info_ = info;
  // The robot's IP address.
  // TODO: Check if parameters need to be member properties or can be local variables
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

  // actuator_count_ = base_.GetActuatorCount().count();
  // RCLCPP_INFO(LOGGER, "Actuator count reported by robot is '%lu'", actuator_count_);

  // Initialize state and command vectors

  // TODO: Finish this comment:
  // If this flag is set, the controller tries to read the values from the command interfaces on activation. If they have real numeric values, those will be used instead of state interfaces. Therefore it is important set command interfaces to NaN (i.e., std::numeric_limits<double>::quiet_NaN()) or state values when the hardware is started.
  // https://control.ros.org/humble/doc/ros2_controllers/joint_trajectory_controller/doc/parameters.html
  joint_velocities_cmd_.resize(actuator_count_, 0.0);
  joint_positions_cmd_.resize(actuator_count_, 0.0);
  joint_positions_.resize(actuator_count_, 0.0);
  joint_velocities_.resize(actuator_count_, 0.0);
  joint_torques_.resize(actuator_count_, 0.0);

  // set size of the twist interface
  twist_cmd_.resize(6, 0.0);

  // TODO: Verify that the URDF's joint count matches the expected count

  // TODO: Check if expected command interfaces are present
  // for (const hardware_interface::ComponentInfo & joint : info_.joints)
  // {
  //   if (!(joint.command_interfaces[0].name == hardware_interface::HW_IF_POSITION ||
  //         joint.command_interfaces[0].name == hardware_interface::HW_IF_VELOCITY ||
  //         joint.command_interfaces[0].name == hardware_interface::HW_IF_EFFORT))
  //   {
  //     RCLCPP_FATAL(
  //       LOGGER, "Joint '%s' has %s command interface. Expected %s, %s, or %s.", joint.name.c_str(),
  //       joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION,
  //       hardware_interface::HW_IF_VELOCITY, hardware_interface::HW_IF_EFFORT);
  //     return CallbackReturn::ERROR;
  //   }

  //   if (!(joint.state_interfaces[0].name == hardware_interface::HW_IF_POSITION ||
  //         joint.state_interfaces[0].name == hardware_interface::HW_IF_VELOCITY ||
  //         joint.state_interfaces[0].name == hardware_interface::HW_IF_EFFORT))
  //   {
  //     RCLCPP_FATAL(
  //       LOGGER, "Joint '%s' has %s state interface. Expected %s, %s, or %s.", joint.name.c_str(),
  //       joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION,
  //       hardware_interface::HW_IF_VELOCITY, hardware_interface::HW_IF_EFFORT);
  //     return CallbackReturn::ERROR;
  //   }
  // }

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

hardware_interface::CallbackReturn
KortexHardwareInterface::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_INFO(LOGGER, "Activating Kortex3 Hardware Interface...");

  try
  {
    /* code */

    // Create MQTT connection for low-frequency commands.
    router_mqtt_ = std::make_shared<k_api::RouterMQTT>(robot_ip_, mqtt_port_);
    router_mqtt_->SpinProcess(std::chrono::milliseconds{ 1 });
    session_mqtt_ = std::make_shared<k_api::Session::SessionClient>(router_mqtt_.get());

    base_ = std::make_shared<k_api::Base::BaseClient>(router_mqtt_.get());

    //

    // Start connections
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

    // program_runner_ = std::make_shared<k_api::ProgramRunner::ProgramRunnerClient>(router_mqtt_.get());
    // protection_zone_ = std::make_shared<k_api::ProtectionZone::ProtectionZoneClient>(router_mqtt_.get());

    // Single level servoing and Monitored Stop on startup
    set_servoing_mode(k_api::Base::SINGLE_LEVEL_SERVOING);
    change_operating_mode(k_api::Common::OPERATING_MODE_MONITORED_STOP);

    // change_operating_mode(k_api::Common::OPERATING_MODE_HOLD_TO_RUN);
    // set_servoing_mode(k_api::Base::LOW_LEVEL_SERVOING);

    // last_operating_mode_ = k_api::Common::OPERATING_MODE_AUTO;
    // set_servoing_mode(k_api::Base::LOW_LEVEL_SERVOING);

    // first read
    auto base_feedback = base_cyclic_->RefreshFeedback();

    // Add each actuator to the base_command_ and set the command to its current position
    for (std::size_t i = 0; i < actuator_count_; i++)
    {
      base_command_.add_actuators()->set_position(base_feedback.actuators(i).position());
    }

    feedback_ = base_cyclic_->RefreshFeedback();

    // rampInit();

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
KortexHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_INFO(LOGGER, "Deactivating Kortex Hardware Interface...");

  // 1. set back the servoing mode to Single Level Servoing and Operating mode to Monitored Stop
  set_servoing_mode(k_api::Base::SINGLE_LEVEL_SERVOING);
  change_operating_mode(k_api::Common::OPERATING_MODE_MONITORED_STOP);

  // 2. Close API sessions
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

  // 3. Deactivate the router and cleanly disconnect from the transport object
  if (router_mqtt_)
  {
    // router_mqtt_->SetActivationStatus(false);
    router_mqtt_->SpinProcess(std::chrono::milliseconds{ 0 });
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

  // 4. Shutdown grippers
  // if (!gripper_a_.joint_name_.empty())
  // {
  //   gripper_a_.shutdown(gripper_mtx_);
  // }
  // if (!gripper_b_.joint_name_.empty())
  // {
  //   gripper_b_.shutdown(gripper_mtx_);
  // }

  // 5. Memory handling
  // delete k_api_twist_;
  // delete gripper_motor_command_;

  RCLCPP_INFO(LOGGER, "Kortex Hardware Interface deactivated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> KortexHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  std::vector<std::string> arm_joint_names;

  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    // RCLCPP_DEBUG(LOGGER, "export_state_interfaces for joint: %s", info_.joints[i].name.c_str());
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
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(arm_joint_names[i], hardware_interface::HW_IF_EFFORT, &joint_torques_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> KortexHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  std::vector<std::string> arm_joint_names;

  for (size_t i = 0; i < info_.joints.size(); i++)
  {
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
  // command_interfaces.emplace_back(
  //   hardware_interface::CommandInterface("tcp", "twist.linear.x", &twist_cmd_[0]));
  // command_interfaces.emplace_back(
  //   hardware_interface::CommandInterface("tcp", "twist.linear.y", &twist_cmd_[1]));
  // command_interfaces.emplace_back(
  //   hardware_interface::CommandInterface("tcp", "twist.linear.z", &twist_cmd_[2]));
  // command_interfaces.emplace_back(
  //   hardware_interface::CommandInterface("tcp", "twist.angular.x", &twist_cmd_[3]));
  // command_interfaces.emplace_back(
  //   hardware_interface::CommandInterface("tcp", "twist.angular.y", &twist_cmd_[4]));
  // command_interfaces.emplace_back(
  //   hardware_interface::CommandInterface("tcp", "twist.angular.z", &twist_cmd_[5]));

  return command_interfaces;
}

hardware_interface::return_type KortexHardwareInterface::prepare_command_mode_switch(
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
        continue;
        // not supporting effort command interface
        //              start_modes_.emplace_back(hardware_interface::HW_IF_EFFORT);
        RCLCPP_ERROR(LOGGER, "KortexHardwareInterface does not support effort command interface!");
      }
    }
    if ((key == "tcp/twist.linear.x") || (key == "tcp/twist.linear.y") || (key == "tcp/twist.linear.z") ||
        (key == "tcp/twist.angular.x") || (key == "tcp/twist.angular.y") || (key == "tcp/twist.angular.z"))
    {
      stop_modes_.emplace_back(StopStartInterface::STOP_TWIST);
    }
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
        continue;
        RCLCPP_ERROR(LOGGER,
                     "KortexHardwareInterface does not support effort command "
                     "interface!");
      }
    }
    if ((key == "tcp/twist.linear.x") || (key == "tcp/twist.linear.y") || (key == "tcp/twist.linear.z") ||
        (key == "tcp/twist.angular.x") || (key == "tcp/twist.angular.y") || (key == "tcp/twist.angular.z"))
    {
      start_modes_.emplace_back(StopStartInterface::START_TWIST);
    }
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
  if (!start_modes_.empty() &&
      (std::find(start_modes_.begin(), start_modes_.end(), StopStartInterface::START_VEL) != start_modes_.end()))
  {
    start_joint_velocity_control_mode_ = true;
  }
  if (!start_modes_.empty() &&
      std::find(start_modes_.begin(), start_modes_.end(), StopStartInterface::START_TWIST) != start_modes_.end())
  {
    start_twist_control_mode_ = true;
  }
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

  // TODO: Currently, if an interface is set to be started while the corresponding control mode is
  // already running, preform_command_mode_switch will reset that control mode. That is the case,
  // for example, when switching between joint_trajectory_controller and cartesian_motion_controller
  // that are both using low level control. I must decide if perform_command_mode_switch should
  // reset the control mode or do nothing.

  return ret_val;
}

hardware_interface::return_type KortexHardwareInterface::perform_command_mode_switch(
    const std::vector<std::string>& /*start_interfaces*/, const std::vector<std::string>& /*stop_interfaces*/)
{
  hardware_interface::return_type ret_val = hardware_interface::return_type::OK;

  if (stop_low_level_control_mode_)
  {
    low_level_control_mode_running_ = false;
    joint_positions_cmd_ = joint_positions_;
  }
  if (stop_joint_velocity_control_mode_)
  {
    joint_velocity_control_mode_running_ = false;
    joint_velocities_cmd_ = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  }
  if (stop_twist_control_mode_)
  {
    twist_control_mode_running_ = false;
    twist_cmd_ = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  }

  if (start_low_level_control_mode_)
  {
    change_operating_mode(k_api::Common::OPERATING_MODE_AUTO);
    set_servoing_mode(k_api::Base::LOW_LEVEL_SERVOING);
    joint_velocity_control_mode_running_ = false;
    twist_control_mode_running_ = false;
    joint_positions_cmd_ = joint_positions_;
    low_level_control_mode_running_ = true;
    // refresh feedback
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

hardware_interface::return_type KortexHardwareInterface::read(const rclcpp::Time& /*time*/,
                                                              const rclcpp::Duration& /*period*/)
{
  try
  {
    // feedback_ = base_cyclic_->RefreshFeedback();

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
    // in_fault_ = true;
    RCLCPP_ERROR(LOGGER, "Robot feedback error: %s", ex.what());
    // RCLCPP_ERROR(LOGGER, "To recover, call the /kortex3_hardware/clear_faults service.");
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type KortexHardwareInterface::write(const rclcpp::Time& /*time*/,
                                                               const rclcpp::Duration& /*period*/)
{
  if (block_write_)
  {
    // feedback_ = base_cyclic_->RefreshFeedback();
    return hardware_interface::return_type::OK;
  }

  if (!in_fault_)
  {
    if (low_level_control_mode_running_)
    {
      if (arm_mode_ == k_api::Base::ServoingMode::LOW_LEVEL_SERVOING)
      {
        sendJointPositionCommands();
      }
    }
    else if (joint_velocity_control_mode_running_)
    {
      if (arm_mode_ == k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING)
      {
        sendJointSpeedsCommand();
      }
    }
    else if (twist_control_mode_running_)
    {
      if (arm_mode_ == k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING)
      {
        // sendTwistCommand();
      }
    }
    else
    {
      // RCLCPP_DEBUG(LOGGER, "No control mode active!");
    }

    if (arm_mode_ == k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING)
    {
      // Twist controller active
      if (joint_velocity_control_mode_running_)
      {
        // twist control
        // sendJointSpeedsCommand();
      }
      else
      {
        // Keep alive mode - no controller active
        // RCLCPP_DEBUG(LOGGER, "No controller active in SINGLE_LEVEL_SERVOING mode!");
      }

      // gripper control
      // sendGripperCommand(
      //   arm_mode_, gripper_command_position_, gripper_speed_command_, gripper_force_command_);
      // read after write in twist mode
      // feedback_ = base_cyclic_->RefreshFeedback();
    }
    // else if (
    //   (arm_mode_ == k_api::Base::ServoingMode::LOW_LEVEL_SERVOING) &&
    //   (feedback_.base().active_state() == k_api::Common::ARMSTATE_SERVOING_LOW_LEVEL))
    else if (arm_mode_ == k_api::Base::ServoingMode::LOW_LEVEL_SERVOING)
    {
      // Per joint controller active

      // gripper control
      // sendGripperCommand(
      //   arm_mode_, gripper_command_position_, gripper_speed_command_, gripper_force_command_);

      if (low_level_control_mode_running_)
      {
        // send commands to the joints
        // sendJointPositionCommands();
      }
      else
      {
        // Keep alive mode - no controller active
        // feedback_ = base_cyclic_->RefreshFeedback();
        // RCLCPP_DEBUG(LOGGER, "No controller active in LOW_LEVEL_SERVOING mode !");
      }
    }
    else
    {
      // Keep alive mode - no controller active
      // feedback_ = base_cyclic_->RefreshFeedback();
      RCLCPP_DEBUG(LOGGER,
                   "Fault was not recognized on the robot but combination of Control Mode and Active State "
                   "are not supported!");
    }
  }
  else
  {
    // this is needed when the robot was faulted
    // so we can internally conclude it is not faulted anymore
    // feedback_ = base_cyclic_->RefreshFeedback();
  }

  return hardware_interface::return_type::OK;
}

void KortexHardwareInterface::change_operating_mode(const k_api::Common::OperatingModeType& mode)
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
    base_->SelectOperatingMode(mode_selection_);
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

void KortexHardwareInterface::set_servoing_mode(const k_api::Base::ServoingMode& mode)
{
  // The possible servoing modes of the robots are:

  // UNSPECIFIED_SERVOING_MODE (0):    Unspecified servoing mode
  // SINGLE_LEVEL_SERVOING (2):        Single level servoing mode (high-level)
  // LOW_LEVEL_SERVOING (3):           Low level servoing mode
  // BYPASS_SERVOING (4):              Bypass servoing mode

  try
  {
    servoing_mode_info_.set_servoing_mode(mode);
    base_->SetServoingMode(servoing_mode_info_);
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

void KortexHardwareInterface::sendJointSpeedsCommand()
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
    base_->SendJointSpeedsCommand(joint_speeds);

    // k_api::Base::Action action;
    // action.set_name("ros2_control_velocity_command");
    // auto *js = action.mutable_send_joint_speeds();

    // // Convert joint velocities from rad/s (ROS) to deg/s (Kortex API).
    // for (size_t i = 0; i < actuator_count_; ++i) {
    //   auto &sp = *js->add_joint_speeds();
    //   sp.set_joint_identifier(i);
    //   sp.set_value(static_cast<float>(joint_velocities_cmd_[i] * 180.0 / M_PI));
    // }
    // base_->ExecuteAction(action);
  }
  catch (const k_api::KDetailedException& e)
  {
    RCLCPP_ERROR(LOGGER, "Unexpected fault during write(): %s", e.what());
    RCLCPP_ERROR_STREAM(LOGGER, "Error sub-code: " << k_api::SubErrorCodes_Name(
                                    k_api::SubErrorCodes((e.getErrorInfo().getError().error_sub_code()))));

    // return hardware_interface::return_type::ERROR;
  }
}

void KortexHardwareInterface::sendJointPositionCommands()
{
  // Incrementing identifier ensures actuators can reject out of time frames
  // base_command_.set_frame_id(base_command_.frame_id() + 1);
  // if (base_command_.frame_id() > 65535)
  //   base_command_.set_frame_id(0);

  // // update the command for each joint
  // for (size_t i = 0; i < actuator_count_; i++)
  // {
  //   base_command_.mutable_actuators(static_cast<int>(i))
  //       ->set_position(static_cast<float>(joint_positions_cmd_[i] * 180.0 / M_PI));
  //   // base_command_.mutable_actuators(static_cast<int>(i))->set_velocity(0.0f);
  //   base_command_.mutable_actuators(static_cast<int>(i))->set_flags(0);
  //   base_command_.mutable_actuators(static_cast<int>(i))->set_command_id(base_command_.frame_id());
  // }

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
    // actuator->set_velocity(joint_velocities_[i] * 180.0 / M_PI);
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

// void KortexMultiInterfaceHardware::sendTwistCommand()
// {
//   k_api_twist_->set_linear_x(static_cast<float>(twist_cmd_[0]));
//   k_api_twist_->set_linear_y(static_cast<float>(twist_cmd_[1]));
//   k_api_twist_->set_linear_z(static_cast<float>(twist_cmd_[2]));
//   k_api_twist_->set_angular_x(static_cast<float>(twist_cmd_[3]));
//   k_api_twist_->set_angular_y(static_cast<float>(twist_cmd_[4]));
//   k_api_twist_->set_angular_z(static_cast<float>(twist_cmd_[5]));
//   base_.SendTwistCommand(k_api_twist_command_);
// }

bool KortexHardwareInterface::rampInit()
{
  uint32_t moving_actuator = 0;
  bool clockwise = true;
  float velocity_deg_per_sec = 3.0f;
  const float amplitude_deg = 10.0f;              // Always move 10 degrees
  const float acceleration_deg_per_sec2 = 25.0f;  // Accel/decel at 25 deg/s^2
  const float deceleration_deg_per_sec2 = 25.0f;

  RCLCPP_INFO(LOGGER, "Starting cyclic loop test (trapezoidal velocity profile):");
  RCLCPP_INFO(LOGGER, "  Servoing mode: %s (%u)", k_api::Base::ServoingMode_Name(arm_mode_).c_str(), arm_mode_);
  RCLCPP_INFO(LOGGER, "  Actuator: %u", moving_actuator);
  RCLCPP_INFO(LOGGER, "  Direction: %s", clockwise ? "clockwise" : "counter-clockwise");
  RCLCPP_INFO(LOGGER, "  Target velocity: %.2f deg/s", velocity_deg_per_sec);
  RCLCPP_INFO(LOGGER, "  Total angle: %.2f deg", amplitude_deg);
  RCLCPP_INFO(LOGGER, "  Fixed acceleration: %.2f deg/s^2", acceleration_deg_per_sec2);
  RCLCPP_INFO(LOGGER, "  Fixed deceleration: %.2f deg/s^2", deceleration_deg_per_sec2);

  // Get initial feedback to establish starting positions
  // Send multiple synchronization cycles to ensure stable communication
  RCLCPP_INFO(LOGGER, "Synchronizing position (sending hold commands)...");
  Kinova::Api::BaseCyclic::Feedback initial_feedback;

  // Send 30 hold-position cycles to synchronize.
  // IMPORTANT: Send hold commands from cycle 0, not just RefreshFeedback.
  // In LOW_LEVEL mode the actuator needs a BaseCyclic position setpoint immediately
  // after mode transition. Delaying commands (RefreshFeedback-only cycles) leaves
  // the actuator without a setpoint, causing Code 50030 when the first Refresh()
  // command eventually arrives.
  for (int sync_cycle = 0; sync_cycle < 30; sync_cycle++)
  {
    try
    {
      // Get current feedback first, then command that exact position.
      Kinova::Api::BaseCyclic::Feedback current_sync_feedback;
      current_sync_feedback = base_cyclic_->RefreshFeedback();

      // Log initial positions on first cycle
      if (sync_cycle == 0)
      {
        RCLCPP_INFO(LOGGER, "Initial positions captured (cycle 0):");
        for (int i = 0; i < current_sync_feedback.actuators_size(); i++)
        {
          RCLCPP_INFO(LOGGER, "  Act %d: %.3f deg", i, current_sync_feedback.actuators(i).position());
        }
      }

      // Build and send hold command using the position we JUST received
      Kinova::Api::BaseCyclic::Command hold_cmd;
      hold_cmd.set_frame_id(sync_cycle);
      for (int i = 0; i < current_sync_feedback.actuators_size(); i++)
      {
        auto* actuator = hold_cmd.add_actuators();
        actuator->set_flags(0);
        actuator->set_position(current_sync_feedback.actuators(i).position());
        actuator->set_velocity(0.0f);
      }

      if (sync_cycle == 0)
      {
        RCLCPP_INFO(LOGGER, "First hold commands sent (cycle 0) using fresh feedback positions");
      }

      current_sync_feedback = base_cyclic_->Refresh(hold_cmd);

      // Update feedback for next cycle
      initial_feedback = current_sync_feedback;
      std::this_thread::sleep_for(std::chrono::milliseconds(1));  // 1ms between sync cycles
    }
    catch (Kinova::Api::KDetailedException& ex)
    {
      RCLCPP_ERROR(LOGGER, "Error during synchronization cycle %d: %s", sync_cycle, ex.what());
      return false;
    }
  }
  RCLCPP_INFO(LOGGER, "Position synchronized");

  // Store initial joint positions and initialize target positions
  std::vector<float> initial_positions;
  std::vector<float> target_positions;  // Track commanded positions independently
  RCLCPP_INFO(LOGGER, "Initial joint positions:");
  if (moving_actuator >= (uint32_t)initial_feedback.actuators_size())
  {
    RCLCPP_ERROR(LOGGER, "Error: actuator_index %u out of range (robot has %d actuators, valid range: 0-%d)", moving_actuator,
           initial_feedback.actuators_size(), initial_feedback.actuators_size() - 1);
    return false;
  }

  for (uint32_t i = 0; i < initial_feedback.actuators_size(); i++)
  {
    float pos = initial_feedback.actuators(i).position();
    initial_positions.push_back(pos);
    target_positions.push_back(pos);  // Start at current position
    if (i == moving_actuator)
    {
      RCLCPP_INFO(LOGGER, "  Joint %u: %.2f deg", i, pos);
    }
  }

  // Calculate trapezoidal (or triangular) profile parameters
  const float direction = clockwise ? 1.0f : -1.0f;
  const float total_angle_to_move = fabs(amplitude_deg);
  const float accel = fabs(acceleration_deg_per_sec2);

  // Clamp peak velocity to what is physically reachable within the given amplitude.
  // If the requested velocity is too high, the profile becomes triangular (no constant
  // velocity phase): the robot accelerates to the clamped peak then immediately decelerates.
  const float max_reachable_velocity = sqrtf(accel * total_angle_to_move);
  float target_velocity = fabs(velocity_deg_per_sec);
  if (target_velocity > max_reachable_velocity)
  {
    RCLCPP_INFO(LOGGER, "  Note: Peak velocity clamped %.2f -> %.2f deg/s (triangular profile, amplitude too small)",
           target_velocity, max_reachable_velocity);
    target_velocity = max_reachable_velocity;
  }

  // Calculate time durations for each phase
  const float accel_time = target_velocity / accel;
  const float decel_time = accel_time;  // Symmetric profile

  // Distance covered during acceleration and deceleration
  const float accel_distance = 0.5f * accel * accel_time * accel_time;  // d = 0.5*a*t²
  const float decel_distance = accel_distance;
  const float const_velocity_distance = total_angle_to_move - accel_distance - decel_distance;
  const float const_time = (const_velocity_distance > 0.0f) ? (const_velocity_distance / target_velocity) : 0.0f;

  RCLCPP_INFO(LOGGER, "  Accel time: %.2f s (distance: %.2f deg)", accel_time, accel_distance);
  RCLCPP_INFO(LOGGER, "  Const velocity time: %.2f s (distance: %.2f deg)", const_time, const_velocity_distance);
  RCLCPP_INFO(LOGGER, "  Decel time: %.2f s (distance: %.2f deg)", decel_time, decel_distance);

  // Timing parameters
  const uint32_t CYCLE_TIME_US = 1000;  // 1ms = 1kHz target
  const float WARMUP_TIME_SEC = 0.5f;   // 500ms warmup (increased for stability after brake release)

  RCLCPP_INFO(LOGGER, "Starting cyclic loop at 1kHz (1ms per cycle)...");
  RCLCPP_INFO(LOGGER, "Warmup period: %.0f ms", WARMUP_TIME_SEC * 1000.0f);
  RCLCPP_INFO(LOGGER, "Will stop automatically 1s after motion completes");

  uint32_t cycle_count = 0;
  bool running = true;
  auto test_start_time = std::chrono::high_resolution_clock::now();
  float last_print_time = 0.0f;
  float angle_moved = 0.0f;           // Track cumulative angle moved
  uint32_t complete_hold_cycles = 0;  // Cycles spent holding after motion complete

  // Store current feedback for next cycle
  Kinova::Api::BaseCyclic::Feedback current_feedback = initial_feedback;

  while (running)
  {
    auto cycle_start = std::chrono::high_resolution_clock::now();

    // Calculate actual elapsed time
    float elapsed_time_sec =
        std::chrono::duration_cast<std::chrono::microseconds>(cycle_start - test_start_time).count() / 1000000.0f;

    // Prepare command
    Kinova::Api::BaseCyclic::Command command;
    command.set_frame_id(cycle_count);

    // Calculate target velocity and acceleration based on time (trapezoidal profile)
    // Use angle_moved only as stopping condition
    float current_velocity;
    float current_accel;                                     // Current acceleration for position calculation
    float motion_time = elapsed_time_sec - WARMUP_TIME_SEC;  // Time since motion started

    if (elapsed_time_sec < WARMUP_TIME_SEC)
    {
      // Warmup phase - no motion
      current_velocity = 0.0f;
      current_accel = 0.0f;
    }
    else if (motion_time >= 0.0f && motion_time < 0.001f)
    {
      // Just started motion - print trace
      RCLCPP_INFO(LOGGER, "*** MOTION STARTED - Robot should be moving now! ***");
      RCLCPP_INFO(LOGGER, "  Time: %.3f s, Angle moved: %.2f deg", elapsed_time_sec, angle_moved);
      current_velocity = accel * motion_time;
      current_accel = accel;
    }
    else if (angle_moved >= total_angle_to_move)
    {
      // Motion complete - hold briefly then exit
      current_velocity = 0.0f;
      current_accel = 0.0f;
      complete_hold_cycles++;
      if (complete_hold_cycles >= 1000)  // 1 second hold then exit
      {
        running = false;
      }
    }
    else if (motion_time < accel_time)
    {
      // Acceleration phase: v = a*t
      current_velocity = accel * motion_time;
      current_accel = accel;  // Positive acceleration
    }
    else if (motion_time < (accel_time + const_time))
    {
      // Constant velocity phase
      current_velocity = target_velocity;
      current_accel = 0.0f;  // No acceleration
    }
    else if (motion_time < (accel_time + const_time + decel_time))
    {
      // Deceleration phase: v = v_target - a*(t - t_decel_start)
      float decel_elapsed = motion_time - accel_time - const_time;
      current_velocity = target_velocity - accel * decel_elapsed;
      if (current_velocity < 0.0f)
        current_velocity = 0.0f;
      current_accel = -accel;  // Negative acceleration (deceleration)
    }
    else
    {
      // Time-based motion complete - hold briefly then exit
      current_velocity = 0.0f;
      current_accel = 0.0f;
      complete_hold_cycles++;
      if (complete_hold_cycles >= 1000)  // 1 second hold then exit
      {
        running = false;
      }
    }

    // Note: TCP velocity limiting is now handled by Kontrol's energy-based limiting
    // in LowLevelPassthroughMode. No testclient-side limiting needed.

    // Build commands for all actuators
    // Only the first num_actuators will move, rest hold position
    for (uint32_t i = 0; i < current_feedback.actuators_size(); i++)
    {
      auto* actuator = command.add_actuators();

      // Set flags to 0 (no special flags needed for LOW_LEVEL mode)
      actuator->set_flags(0);

      if (i == moving_actuator)
      {
        // This actuator should move
        // Compute position increment using velocity-only: Δx = v*Δt
        // (Removed acceleration term to prevent tracking errors)
        const float dt = 0.001f;  // 1ms cycle time @ 1kHz
        float position_increment = direction * current_velocity * dt;

        // Update target position for this actuator
        target_positions[i] += position_increment;

        // Set position and velocity commands
        actuator->set_position(target_positions[i]);
        actuator->set_velocity(fabs(current_velocity));

        angle_moved += fabs(position_increment);
      }
      else
      {
        // This actuator should hold position (use tracked target, not feedback)
        actuator->set_position(target_positions[i]);
        actuator->set_velocity(0.0f);
      }
    }

    // Send command and get feedback
    try
    {
      Kinova::Api::BaseCyclic::Feedback new_feedback = base_cyclic_->Refresh(command);
      current_feedback = new_feedback;  // Store for next cycle

      // Print status every second
      if (elapsed_time_sec - last_print_time >= 1.0f)
      {
        auto cycle_end = std::chrono::high_resolution_clock::now();
        auto cycle_time_us = std::chrono::duration_cast<std::chrono::microseconds>(cycle_end - cycle_start).count();
        float cycle_rate_hz = cycle_count / elapsed_time_sec;

        // Determine current phase based on time
        const char* phase = "warmup";
        if (elapsed_time_sec >= WARMUP_TIME_SEC)
        {
          float motion_time_status = elapsed_time_sec - WARMUP_TIME_SEC;
          if (angle_moved >= total_angle_to_move || motion_time_status >= (accel_time + const_time + decel_time))
          {
            phase = "complete";
          }
          else if (motion_time_status < accel_time)
          {
            phase = "accel";
          }
          else if (motion_time_status < (accel_time + const_time))
          {
            phase = "const";
          }
          else
          {
            phase = "decel";
          }
        }

        RCLCPP_INFO(LOGGER, 
            "Time: %.1fs [%s], Cycles: %u (%.0f Hz), Cycle: %ld us, Moved: %.2f/%.2f deg, J%u: %.2f deg, V%u: %.2f "
            "deg/s",
            elapsed_time_sec, phase, cycle_count, cycle_rate_hz, cycle_time_us, angle_moved, total_angle_to_move,
            moving_actuator,
            current_feedback.actuators_size() > (int)moving_actuator ?
                current_feedback.actuators(moving_actuator).position() :
                0.0f,
            moving_actuator,
            current_feedback.actuators_size() > (int)moving_actuator ?
                current_feedback.actuators(moving_actuator).velocity() :
                0.0f);

        last_print_time = elapsed_time_sec;

        // Check for faults
        if (current_feedback.base().fault_bank_a() != 0 || current_feedback.base().fault_bank_b() != 0)
        {
          RCLCPP_INFO(LOGGER, "FAULT DETECTED! Bank A: 0x%08X, Bank B: 0x%08X", current_feedback.base().fault_bank_a(),
                 current_feedback.base().fault_bank_b());
          running = false;
        }
      }
    }
    catch (Kinova::Api::KDetailedException& ex)
    {
      RCLCPP_ERROR(LOGGER, "Error during cyclic loop (time %.2fs, cycle %u): %s", elapsed_time_sec, cycle_count, ex.what());
      running = false;
      break;
    }

    cycle_count++;

    // Busy-wait to maintain 1kHz rate (1ms per cycle)
    auto cycle_end = std::chrono::high_resolution_clock::now();
    auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(cycle_end - cycle_start).count();

    // Spin until 1ms has elapsed from cycle start
    while (elapsed_us < CYCLE_TIME_US)
    {
      cycle_end = std::chrono::high_resolution_clock::now();
      elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(cycle_end - cycle_start).count();
    }
  }

  auto final_time = std::chrono::high_resolution_clock::now();
  float total_time_sec =
      std::chrono::duration_cast<std::chrono::microseconds>(final_time - test_start_time).count() / 1000000.0f;
  float avg_cycle_rate_hz = cycle_count / total_time_sec;
  float avg_cycle_time_us = (total_time_sec * 1000000.0f) / cycle_count;

  RCLCPP_INFO(LOGGER, "Cyclic loop completed:");
  RCLCPP_INFO(LOGGER, "  Total time: %.2f seconds", total_time_sec);
  RCLCPP_INFO(LOGGER, "  Total cycles: %u", cycle_count);
  RCLCPP_INFO(LOGGER, "  Average rate: %.0f Hz", avg_cycle_rate_hz);
  RCLCPP_INFO(LOGGER, "  Average cycle time: %.0f us", avg_cycle_time_us);

  return true;
}

}  // namespace kortex3_driver

#include "pluginlib/class_list_macros.hpp"

// Registers this class with pluginlib, making it available to the ros2_control controller manager.
PLUGINLIB_EXPORT_CLASS(kortex3_driver::KortexHardwareInterface, hardware_interface::SystemInterface)