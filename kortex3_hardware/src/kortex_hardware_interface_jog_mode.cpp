
#include <chrono>
#include <string>
#include <vector>

#include "kortex3_hardware/kortex_hardware_interface_jog_mode.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace kortex3_driver
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("KortexHardwareInterfaceJogMode");

KortexHardwareInterfaceJogMode::KortexHardwareInterfaceJogMode()
  : mode_selection_(k_api::Common::ModeSelection())
  , servoing_mode_info_(k_api::Base::ServoingModeInformation())
  , actuator_count_(6) // Default, updated from robot during activation.
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
  router_udp_ = std::make_unique<k_api::RouterClient>(
    transport_udp_.get(),
    [](k_api::KError err) {
      RCLCPP_ERROR(LOGGER, "UDP Router error: %s", err.toString().c_str());
    }
  );
  session_udp_ = std::make_unique<k_api::SessionManager>(router_udp_.get());

  base_cyclic_ = std::make_shared<k_api::BaseCyclic::BaseCyclicClient>(router_udp_.get());
}

hardware_interface::CallbackReturn KortexHardwareInterfaceJogMode::on_init(const hardware_interface::HardwareInfo& info)
{
  RCLCPP_INFO(LOGGER, "Initializing Kortex3 Hardware Interface in JOG MODE...");
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
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

  session_inactivity_timeout_ =
    std::stoi(info_.hardware_parameters["session_inactivity_timeout_ms"]);
  if (session_inactivity_timeout_ <= 0)
  {
    RCLCPP_ERROR(LOGGER, "Incorrect session inactivity timeout!");
    return CallbackReturn::ERROR;
  }
  else
  {
    RCLCPP_INFO(LOGGER, "Session inactivity timeout is '%d'", session_inactivity_timeout_);
  }
  connection_inactivity_timeout_ =
    std::stoi(info_.hardware_parameters["connection_inactivity_timeout_ms"]);
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
  joint_velocities_cmd_.resize(actuator_count_, 0.0);
  joint_positions_.resize(actuator_count_, 0.0);
  joint_velocities_.resize(actuator_count_, 0.0);
  joint_torques_.resize(actuator_count_, 0.0);

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
KortexHardwareInterfaceJogMode::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_INFO(LOGGER, "Activating Kortex3 Hardware Interface...");

  try
  {
    /* code */







    // 1. Create MQTT connection for low-frequency commands.
    router_mqtt_ = std::make_shared<k_api::RouterMQTT>(robot_ip_, mqtt_port_);
    router_mqtt_->SpinProcess(std::chrono::milliseconds{1});
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


    // base_mqtt_ = std::make_shared<k_api::Base::BaseClient>(router_mqtt_.get());
    // program_runner_ = std::make_shared<k_api::ProgramRunner::ProgramRunnerClient>(router_mqtt_.get());
    // protection_zone_ = std::make_shared<k_api::ProtectionZone::ProtectionZoneClient>(router_mqtt_.get());


    // TODO: SET OPERATING MODE AND SERVOING MODE
    // 5. Set operating mode to AUTO, then switch to low-level servoing for position control.
    change_operating_mode(k_api::Common::OPERATING_MODE_JOG_MANUAL);
    // last_operating_mode_ = k_api::Common::OPERATING_MODE_AUTO;

    // 6. Switch to LOW_LEVEL_SERVOING so position setpoints are sent via BaseCyclic::Refresh().
    set_servoing_mode(k_api::Base::SINGLE_LEVEL_SERVOING);
    // set_servoing_mode(k_api::Base::LOW_LEVEL_SERVOING);


    // first read
    auto base_feedback = base_cyclic_->RefreshFeedback();


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
KortexHardwareInterfaceJogMode::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_INFO(LOGGER, "Deactivating Kortex Hardware Interface...");


  // 1. set back the servoing mode to Single Level Servoing and Operating mode to Monitored Stop
  set_servoing_mode(k_api::Base::SINGLE_LEVEL_SERVOING);
  change_operating_mode(k_api::Common::OPERATING_MODE_MONITORED_STOP);

  // 2. Close API sessions
  if (session_udp_)
  {
    try { session_udp_->CloseSession(); }
    catch (const std::exception &e) {
      RCLCPP_ERROR(LOGGER, "Error closing UDP session: %s", e.what());
    }
  }
  if (session_mqtt_)
  {
    try { session_mqtt_->CloseSession(); }
    catch (const std::exception &e) {
      RCLCPP_ERROR(LOGGER, "Error closing MQTT session: %s", e.what());
    }
  }

  // 3. Deactivate the router and cleanly disconnect from the transport object
  if (router_mqtt_)
  {
    // router_mqtt_->SetActivationStatus(false);
    router_mqtt_->SpinProcess(std::chrono::milliseconds{0});
  }
  if (router_udp_)
  {
    router_udp_->SetActivationStatus(false);
  }
  if (transport_udp_)
  {
    try { transport_udp_->disconnect(); }
    catch (const std::exception &e) {
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

std::vector<hardware_interface::StateInterface> KortexHardwareInterfaceJogMode::export_state_interfaces()
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
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      arm_joint_names[i], hardware_interface::HW_IF_EFFORT, &joint_torques_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> KortexHardwareInterfaceJogMode::export_command_interfaces()
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
  }

  return command_interfaces;
}

hardware_interface::return_type KortexHardwareInterfaceJogMode::read(const rclcpp::Time& /*time*/,
                                                              const rclcpp::Duration& /*period*/)
{
  try 
  {
    feedback_ = base_cyclic_->RefreshFeedback();

    for (size_t i = 0; i < feedback_.actuators_size() && i < actuator_count_; ++i) {
      const auto &act = feedback_.actuators(i);
      joint_positions_[i]  = act.position() * M_PI / 180.0;
      joint_velocities_[i] = act.velocity() * M_PI / 180.0;
      joint_torques_[i]    = act.torque();
    }

  }
  catch (const k_api::KDetailedException &ex) {
    // in_fault_ = true;
    RCLCPP_ERROR(LOGGER, "Robot feedback error: %s", ex.what());
    // RCLCPP_ERROR(LOGGER, "To recover, call the /kortex3_hardware/clear_faults service.");
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type KortexHardwareInterfaceJogMode::write(const rclcpp::Time& /*time*/,
                                                               const rclcpp::Duration& /*period*/)
{
  if (!in_fault_)
  {
    if (arm_mode_ == k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING)
    {
      sendJointSpeedsCommand();

      // gripper control
      // sendGripperCommand(
      //   arm_mode_, gripper_command_position_, gripper_speed_command_, gripper_force_command_);
      // read after write in twist mode
      // feedback_ = base_cyclic_->RefreshFeedback();
    }
    // else if (
    //   (arm_mode_ == k_api::Base::ServoingMode::LOW_LEVEL_SERVOING) &&
    //   (feedback_.base().active_state() == k_api::Common::ARMSTATE_SERVOING_LOW_LEVEL))
    else
    {
      // Keep alive mode - no controller active
      // feedback_ = base_cyclic_->RefreshFeedback();
      RCLCPP_DEBUG(
        LOGGER,
        "Fault was not recognized on the robot but Control Mode is not supported!");
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

void KortexHardwareInterfaceJogMode::change_operating_mode(
  const k_api::Common::OperatingModeType &mode)
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
    RCLCPP_INFO(LOGGER, "Operating mode set to %s.",
      k_api::Common::OperatingModeType_Name(mode).c_str());
  }
  catch (const k_api::KDetailedException &ex)
  {
    RCLCPP_ERROR(LOGGER, "Failed to change operating mode: %s", ex.what());
  }
}

void KortexHardwareInterfaceJogMode::set_servoing_mode(
  const k_api::Base::ServoingMode &mode)
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
    RCLCPP_INFO(LOGGER, "Servoing mode set to %s.",
      k_api::Base::ServoingMode_Name(mode).c_str());
  }
  catch (const k_api::KDetailedException &ex)
  {
    RCLCPP_ERROR(LOGGER, "Failed to set servoing mode: %s", ex.what());
  }
}

void KortexHardwareInterfaceJogMode::sendJointSpeedsCommand()
{
  try {
    k_api::Base::JointSpeeds joint_speeds;
    for (size_t i = 0; i < actuator_count_; ++i) {
      auto joint_speed = joint_speeds.add_joint_speeds();
      joint_speed->set_joint_identifier(i);
      joint_speed->set_value(static_cast<float>(joint_velocities_cmd_[i] * 180.0 / M_PI));
    }
    base_->SendJointSpeedsCommand(joint_speeds);
  }
  catch (const k_api::KDetailedException &e) {
    RCLCPP_ERROR(LOGGER, "Unexpected fault during write(): %s", e.what());
    // return hardware_interface::return_type::ERROR;
  }
}

}  // namespace kortex3_driver

#include "pluginlib/class_list_macros.hpp"

// Registers this class with pluginlib, making it available to the ros2_control controller manager.
PLUGINLIB_EXPORT_CLASS(kortex3_driver::KortexHardwareInterfaceJogMode, hardware_interface::SystemInterface)