#ifndef KORTEX_HARDWARE_INTERFACE
#define KORTEX_HARDWARE_INTERFACE

// System Headers
#include <atomic>
#include <limits>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

// Gripper
#include "kortex3_hardware/gripper_controller.hpp"

// Kortex 3 API Headers
#include "SessionClientRpc.h"
#include "RouterMQTT.h"

#include "TransportClientUdp.h"
#include "RouterClient.h"
#include "SessionManager.h"

#include "BaseClientRpc.h"
#include "BaseCyclicClientRpc.h"
#include "SafetyFunctionsClientRpc.h"

#include "Base.pb.h"

// ROS 2 Headers
#include "rclcpp/rclcpp.hpp"
// #include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

namespace k_api = Kinova::Api;

namespace kortex3_driver
{
enum class StopStartInterface
{
  NONE,
  STOP_POS,
  STOP_VEL,
  STOP_TWIST,
  STOP_GRIPPER,
  STOP_FAULT_CTRL,
  START_POS,
  START_VEL,
  START_TWIST,
  START_GRIPPER,
  START_FAULT_CTRL,
};
class Kortex3HardwareInterfaceLowLevel : public hardware_interface::SystemInterface
{
public:
  // Class Constructor
  Kortex3HardwareInterfaceLowLevel();

  // Standard ros2_control lifecycle methods
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  // Methods to export hardware interfaces to the controller manager
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // Methods to prepare for and perform switching to a new command interface
  hardware_interface::return_type prepare_command_mode_switch(const std::vector<std::string>& start_interfaces,
                                                              const std::vector<std::string>& stop_interfaces) final;
  hardware_interface::return_type perform_command_mode_switch(
      const std::vector<std::string>& /*start_interfaces*/, const std::vector<std::string>& /*stop_interfaces*/) final;

  // Core methods for communication with the hardware
  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
  // --- Connection and configuration Parameters ---
  std::string robot_ip_;               ///< IP address of the robot controller.
  std::string username_;               ///< Username for session authentication.
  std::string password_;               ///< Password for session authentication.
  int mqtt_port_;                      ///< Port for low frequency MQTT communication (commands).
  int port_realtime_;                  ///< Port for realtime UDP communication (feedback and commands).
  int session_inactivity_timeout_;     ///< Inactivity period (in milliseconds) allowed before the session times out and closes on its own.
  int connection_inactivity_timeout_;  ///< Inactivity period (in milliseconds) allowed before the robot stops any movements initiated from this session.
  std::string gripper_name_;           ///< Which gripper is being used.
  uint16_t gripper_modbus_id_;         ///< The modbus ID to communicate with the gripper.
  size_t actuator_count_;              ///< Number of actuators to control


  // --- State and Command Buffers ---
  std::vector<double> joint_positions_cmd_;   ///< Buffer for position commands sent via BaseCyclic::Refresh().
  std::vector<double> joint_velocities_cmd_;  ///< Buffer for velocity commands sent via high-level interface.
  std::vector<double> twist_cmd_;             ///< Buffer for twist commands
  std::vector<double> joint_positions_;       ///< Buffer for joint position states.
  std::vector<double> joint_velocities_;      ///< Buffer for joint velocity states.
  std::vector<double> joint_torques_;         ///< Buffer for joint torque states.

  // Monotonically increasing frame counter for BaseCyclic commands.
  int base_command_frame_id_; 
  
  
  // --- Kortex API Objects ---
  std::shared_ptr<k_api::RouterMQTT> router_mqtt_;
  std::shared_ptr<k_api::Session::SessionClient> session_mqtt_;

  std::unique_ptr<k_api::TransportClientUdp> transport_udp_;
  std::unique_ptr<k_api::RouterClient> router_udp_;
  std::unique_ptr<k_api::SessionManager> session_udp_;

  std::shared_ptr<k_api::Base::BaseClient> base_mqtt_;
  std::shared_ptr<k_api::BaseCyclic::BaseCyclicClient> base_cyclic_;
  k_api::BaseCyclic::Feedback feedback_;

  // Changing active controller on the hardware
  k_api::Base::ServoingMode arm_mode_;
  k_api::Common::ModeSelection mode_selection_;
  k_api::Base::ServoingModeInformation servoing_mode_info_;
  k_api::Common::OperatingModeType low_level_operating_mode_;
  k_api::SafetyFunctions::SafetySystemMode low_level_safety_mode_;

  // Required to change the SafetyMode (Reduced vs. Normal)
  std::shared_ptr<k_api::SafetyFunctions::SafetyFunctionsClient> safety_functions_client_;
  k_api::SafetyFunctions::SafetySystem safety_system_;

  // twist temporary command
  k_api::Base::Twist * k_api_twist_;
  k_api::Base::TwistCommand k_api_twist_command_;


  // Mode switching auxiliary vars
  // keeping track of which controller is active so appropriate control mode can be adjusted
  // controller manager sends array of interfaces that should be stopped/started and this is the
  // way to internally collect information on which controller should be stopped and started
  // (different controllers claim different interfaces)
  std::vector<StopStartInterface> stop_modes_;
  std::vector<StopStartInterface> start_modes_;
  // switching auxiliary booleans
  bool stop_low_level_control_mode_;
  bool stop_joint_velocity_control_mode_;
  bool stop_twist_control_mode_;
  bool start_low_level_control_mode_;
  bool start_joint_velocity_control_mode_;
  bool start_twist_control_mode_;
  bool low_level_control_mode_running_;
  bool joint_velocity_control_mode_running_;
  bool twist_control_mode_running_;


  // Fault management
  std::atomic<bool> in_fault_ = false;
  std::atomic<bool> block_write_ = false;

  // --- Gripper ---
  bool use_internal_bus_gripper_comm_;
  GripperController gripper_a_;
  GripperController gripper_b_;
  std::mutex gripper_mtx_;   ///< Shared Modbus bus mutex for all gripper operations.

  // CM-thread-owned buffers exported via hardware interfaces
  double gripper_a_pos_;
  double gripper_a_vel_;
  double gripper_a_cmd_;
  double gripper_b_pos_;
  double gripper_b_vel_;
  double gripper_b_cmd_;

  // Lock-free exchange between the CM update thread and the gripper background thread.
  // CM write() stores to *_cmd_atomic_; CM read() loads from *_pos_atomic_.
  // Background thread reads *_cmd_atomic_ and writes *_pos_atomic_.
  std::atomic<double> gripper_a_cmd_atomic_;
  std::atomic<double> gripper_a_pos_atomic_;
  std::atomic<double> gripper_b_cmd_atomic_;
  std::atomic<double> gripper_b_pos_atomic_;

  std::atomic<bool> gripper_thread_running_;
  std::thread gripper_thread_;
  void gripperThreadLoop();

  // --- Private Helper Methods ---
  void change_operating_mode(const k_api::Common::OperatingModeType& mode);
  void set_servoing_mode(const k_api::Base::ServoingMode& mode);
  void set_safety_system_mode(const k_api::SafetyFunctions::SafetySystemMode& mode);

  void sendJointPositionCommands();
  void sendJointSpeedsCommand();
  void sendTwistCommand();
};

}  // namespace kortex3_driver

#endif  // KORTEX_HARDWARE_INTERFACE