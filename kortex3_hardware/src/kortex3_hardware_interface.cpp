/**
 * @file kortex3_hardware_interface.cpp
 * @brief Implementation of the ros2_control hardware interface for the Kinova Link6 robot.
 * @author Anas Houssaini
 */

#include "kortex3_hardware/kortex3_hardware_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <thread>
#include <string>

namespace kortex3_driver
{
const rclcpp::Logger Kortex3HardwareInterface::LOGGER =
  rclcpp::get_logger("Kortex3HardwareInterface");

Kortex3HardwareInterface::Kortex3HardwareInterface()
  : mqtt_port_(1883),
    udp_feedback_port_(10001),
    actuator_count_(6), // Default, updated from robot during activation.
    in_fault_(false),
    node_ptr_(nullptr),
    gripper_joint_name_("")
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
  if (info_.hardware_parameters.count("mqtt_port"))
  {
    mqtt_port_ = std::stoi(info_.hardware_parameters.at("mqtt_port"));
  }
  if (info_.hardware_parameters.count("udp_feedback_port"))
  {
    udp_feedback_port_ = std::stoi(info_.hardware_parameters.at("udp_feedback_port"));
  }

  // gripper joint name
  gripper_joint_name_ = info_.joints[info_.joints.size()-1].name;
  if (gripper_joint_name_.empty())
  {
    RCLCPP_ERROR(LOGGER, "Gripper joint name is empty!");
  }
  else
  {
    RCLCPP_INFO(LOGGER, "Gripper joint name is '%s'", gripper_joint_name_.c_str());
  }

  // Initialize state and command vectors.
  joint_velocities_cmd_.resize(actuator_count_, 0.0);
  joint_positions_.resize(actuator_count_, 0.0);
  joint_velocities_.resize(actuator_count_, 0.0);
  joint_torques_.resize(actuator_count_, 0.0);
  gripper_command_position_ = std::numeric_limits<double>::quiet_NaN();
  gripper_position_ = std::numeric_limits<double>::quiet_NaN();

  // Verify that the URDF's joint count matches the expected count.
  if (info_.joints.size() != actuator_count_+1)
  {
    RCLCPP_ERROR(LOGGER,
      "URDF configuration error: Expected %zu joints, but got %zu.",
      actuator_count_, info_.joints.size());
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

    // 5. Generate and load robot calibration.
    if (!calibrate_robot())
    {
        RCLCPP_WARN(LOGGER, "Calibration generation failed; using nominal model.");
    }

    // 6. Set the initial operating mode for velocity control.
    change_operating_mode(k_api::Common::OPERATING_MODE_AUTO);
    last_operating_mode_ = k_api::Common::OPERATING_MODE_AUTO;
/*
    // Initialize gripper Modbus connection
    constexpr uint16_t gripper_slave_id = 9; // Robotiq default Modbus ID
    modbus_wrapper_ = std::make_shared<slick::com::ModbusClientWrapper>(router_mqtt_, gripper_slave_id);

    if (modbus_wrapper_->TryInitConnection() != slick::com::ModbusError::Ok) {
      RCLCPP_ERROR(LOGGER, "Failed to connect to Robotiq gripper via Modbus.");
      return hardware_interface::CallbackReturn::ERROR;
    }

    gripper_ = std::make_unique<MyFingerGripper>(modbus_wrapper_);
*/
    // Activate the gripper
    /*
    gripper_->SetActivateRequest();
    gripper_->SendRequest();
    std::this_thread::sleep_for(std::chrono::milliseconds(50)); // Optional short delay
    gripper_->ReadRegister();
    if (!gripper_->GetActivationCompleted()) 
    {
      RCLCPP_WARN(LOGGER, "Gripper not fully activated yet");
    }
    */
    // First read from gripper
    auto opt_gripper_position = readGripperPosition();
    if (!opt_gripper_position.has_value())
    {
      RCLCPP_WARN(LOGGER, "Failed to read gripper position on activation.");
      return hardware_interface::CallbackReturn::ERROR;
    }
    float gripper_initial_position = static_cast<float>(opt_gripper_position.value());
    RCLCPP_INFO(LOGGER, "Gripper initial position is '%f'.", gripper_initial_position);

    //to radians
    gripper_command_position_ = gripper_initial_position;

    sendGripperCommand(gripper_initial_position);

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

    std::lock_guard<std::mutex> lk(gripper_mtx_);
    if (modbus_wrapper_) modbus_wrapper_->CloseConnection();
    gripper_.reset();
    modbus_wrapper_.reset();
    gripper_initialized_ = false;
  }
  catch (const std::exception& ex)
  {
    RCLCPP_ERROR(LOGGER, "Error during deactivation: %s", ex.what());
  }

  // Reset ROS 2 interfaces and pointers.
  set_operating_mode_service_.reset();
  clear_faults_service_.reset();
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
    if (info_.joints[i].name == gripper_joint_name_)
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &gripper_position_));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &gripper_velocity_));
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
    if (info_.joints[i].name == gripper_joint_name_)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &gripper_command_position_));
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

  // read gripper state
  readGripperPosition();
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
      RCLCPP_ERROR(LOGGER, "Robot arm is in FAULT (state=%s).",
                   Kinova::Api::Common::ArmState_Name(new_state).c_str());
      RCLCPP_ERROR(LOGGER, "To recover, call the /kortex3_hardware/clear_faults service.");
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

std::optional<double> Kortex3HardwareInterface::readGripperPosition()
{
  // 1) Rate-limit BEFORE touching the mutex
  const auto now = std::chrono::steady_clock::now();
  if (now < next_gripper_poll_) {
    return gripper_position_;  // return cached value
  }

  // 2) Try to acquire the mutex without blocking the control loop
  std::unique_lock<std::mutex> lk(gripper_mtx_, std::try_to_lock);
  if (!lk.owns_lock()) {
    // Another Modbus op in-flight; try again soon without blocking
    next_gripper_poll_ = now + std::chrono::milliseconds(10);
    return gripper_position_;
  }

  // 3) Lazy init once
  if (!gripper_initialized_) {
    constexpr uint16_t gripper_slave_id = 9;
    modbus_wrapper_ = std::make_shared<slick::com::ModbusClientWrapper>(router_mqtt_, gripper_slave_id);

    if (modbus_wrapper_->TryInitConnection() != slick::com::ModbusError::Ok) {
      RCLCPP_WARN(LOGGER, "Modbus init failed; will retry later.");
      next_gripper_poll_ = now + gripper_poll_period_;
      return std::nullopt;
    }

    gripper_ = std::make_unique<MyFingerGripper>(modbus_wrapper_);
    gripper_initialized_ = true;
  }

  // 4) Read once; if it fails, reconnect and retry once (no sleeps)
  bool ok = gripper_->ReadRegister();
  if (!ok) {
    RCLCPP_WARN(LOGGER, "ReadRegister() failed; reconnecting and retrying once");
    modbus_wrapper_->CloseConnection();
    if (modbus_wrapper_->TryInitConnection() == slick::com::ModbusError::Ok) {
      ok = gripper_->ReadRegister();
    }
  }

  // Set next poll time regardless, so we don’t hammer on failures
  next_gripper_poll_ = now + gripper_poll_period_;

  if (!ok) {
    return std::nullopt;
  }

  // 5) Decode & cache
  const uint8_t raw = gripper_->GetPosition();
  gripper_position_ = static_cast<double>(raw) / 255.0 * 0.81;  // rad
  //std::cout << "Gripper position: " << gripper_position_ << std::endl;
  return gripper_position_;
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
    RCLCPP_ERROR(LOGGER, "Fault during write(): %s", e.what());
    RCLCPP_ERROR(LOGGER, "To recover, call the /kortex3_hardware/clear_faults service.");
    in_fault_ = true;
    return hardware_interface::return_type::ERROR;
  }

  //Gripper command
  sendGripperCommand(gripper_command_position_);

  return hardware_interface::return_type::OK;
}

void Kortex3HardwareInterface::sendGripperCommand(double position_radians)
{
  // --- Gate BEFORE locking to avoid needless contention ---
  const auto now = std::chrono::steady_clock::now();

  // Limit command rate
  if (now < next_gripper_send_) return;

  // Clamp into stroke and skip tiny, redundant updates
  const double clamped = std::clamp(position_radians, 0.0, 0.81);
  if (last_gripper_cmd_pos_ == last_gripper_cmd_pos_ &&   // not NaN
      std::abs(clamped - last_gripper_cmd_pos_) < 0.005)  // ~0.5% of stroke
  {
    return;
  }

  // --- Serialize Modbus access ---
  std::lock_guard<std::mutex> lk(gripper_mtx_);
  if (!gripper_initialized_ || !gripper_) return;

  const uint8_t pos = static_cast<uint8_t>((clamped / 0.81) * 255.0);

  // Best-effort write sequence (no sleeps, no loop blocking)
  gripper_->ClearGoToRequest();
  (void)gripper_->SendRequest();

  gripper_->SetPositionRequest(pos);
  gripper_->SetGoToRequest();
  (void)gripper_->SendRequest();

  // Optionally: quick reconnect + one retry if your API returns a bool
  // if (!gripper_->SendRequest()) {
  //   modbus_wrapper_->CloseConnection();
  //   if (modbus_wrapper_->TryInitConnection() == slick::com::ModbusError::Ok) {
  //     (void)gripper_->SendRequest();
  //   }
  // }

  last_gripper_cmd_pos_ = clamped;
  next_gripper_send_    = now + gripper_cmd_period_;
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
  try {
    auto armState = base_mqtt_->GetArmState();
    if (armState.active_state() == k_api::Common::ARMSTATE_IN_FAULT ||
        armState.active_state() == k_api::Common::ARMSTATE_IN_FAULT_POWERED_OFF)
    {
      // 1. Clear the fault, which puts the arm into RECOVERY state.
      base_mqtt_->ClearFaults();
      RCLCPP_INFO(LOGGER, "Called ClearFaults(), arm should be in RECOVERY.");

      // 2. Exit recovery state to return to OPERATIONAL.
      base_mqtt_->ExitRecoveryState();
      RCLCPP_INFO(LOGGER, "Called ExitRecoveryState(). Waiting for arm to become OPERATIONAL...");

      // 3. Wait for the arm to confirm it is operational.
      const auto timeout = std::chrono::seconds(5);
      auto start = std::chrono::steady_clock::now();
      while (std::chrono::steady_clock::now() - start < timeout) {
        if (base_mqtt_->GetArmState().active_state() == k_api::Common::ARMSTATE_ARM_OPERATIONAL) {
          RCLCPP_INFO(LOGGER, "Arm recovered to OPERATIONAL.");
          break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
      }

      // 4. Reset internal fault flag to resume normal operation.
      in_fault_ = false;
      response->success = true;
      response->message = "Faults cleared and arm recovered to OPERATIONAL.";
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

bool Kortex3HardwareInterface::dump_calibration(const std::string& serial)
{
  try
  {
    // 1. Fetch the calibration data blob from the robot via MQTT.
    auto blob = base_mqtt_->ExportArmCalibration();

    // 2. Determine the path to save the calibration files.
    // This assumes a companion description package (e.g., link6_description).
    auto desc_share = ament_index_cpp::get_package_share_directory("link6_description");
    fs::path cal_dir = fs::path(desc_share) / "calibration";
    fs::create_directories(cal_dir);
    fs::path zip_path = cal_dir / (serial + ".zip");

    // 3. Write the fetched data to a .zip file.
    std::ofstream out(zip_path, std::ios::binary | std::ios::trunc);
    for (auto b : blob.data()) out.put(static_cast<char>(b));
    out.close();
    RCLCPP_INFO(LOGGER, "Calibration ZIP saved to %s", zip_path.c_str());

    // 4. Unzip calib.xml from the archive for the generator script to use.
    // Note: This creates an external dependency on the `unzip` command.
    std::string cmd = "unzip -oq " + zip_path.string() + " calib.xml -d " + cal_dir.string();
    if (std::system(cmd.c_str()) != 0) {
      RCLCPP_WARN(LOGGER, "unzip command failed; XML may already exist or unzip is not installed.");
    }
    return true;
  }
  catch (const std::exception& ex)
  {
    RCLCPP_ERROR(LOGGER, "dump_calibration() failed: %s", ex.what());
    return false;
  }
}

bool Kortex3HardwareInterface::calibrate_robot()
{
  // Kortex 3 does not yet expose a unique serial number, so we use a fixed name.
  const std::string serial = "link6";
  if (!dump_calibration(serial))
  {
      return false;
  }

  // Define paths for all required files and scripts.
  const fs::path share_dir = ament_index_cpp::get_package_share_directory("link6_description");
  const fs::path cal_dir   = share_dir / "calibration";
  const fs::path xml_path  = cal_dir / "calib.xml";
  const fs::path xacro_nom = share_dir / "urdf" / "link6_nominal.xacro";
  const fs::path temp_nom  = cal_dir  / "link6_nominal.urdf";
  const fs::path py_script = share_dir / "scripts" / "calibrated_urdf_generator.py";
  const fs::path out_xacro = share_dir / "urdf"   / "link6_calibrated.xacro";

  // 1. Expand the nominal Xacro to a temporary URDF file.
  // Note: This creates an external dependency on `xacro`.
  std::ostringstream xacro_cmd;
  xacro_cmd << "xacro " << xacro_nom << " -o " << temp_nom;
  if (std::system(xacro_cmd.str().c_str()) != 0)
  {
    RCLCPP_ERROR(LOGGER, "xacro failed while expanding nominal model.");
    return false;
  }

  // 2. Invoke the Python generator script to create the calibrated Xacro file.
  // Note: This creates an external dependency on `python3` and the script itself.
  std::ostringstream python_cmd;
  python_cmd << "python3 " << py_script
             << " --urdf_path "        << temp_nom
             << " --calibration_file " << xml_path
             << " --output_file "      << out_xacro;
  if (std::system(python_cmd.str().c_str()) != 0)
  {
    RCLCPP_WARN(LOGGER, "Calibration generation script failed; will use nominal model.");
    return false;
  }

  RCLCPP_INFO(LOGGER, "Calibrated Xacro written to %s", out_xacro.c_str());
  return true;
}

} // namespace kortex3_driver

#include "pluginlib/class_list_macros.hpp"
// Registers this class with pluginlib, making it available to the ros2_control controller manager.
PLUGINLIB_EXPORT_CLASS(
  kortex3_driver::Kortex3HardwareInterface,
  hardware_interface::SystemInterface)