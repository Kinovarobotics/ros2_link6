#ifndef KORTEX_HARDWARE_INTERFACE_JOG_MODE
#define KORTEX_HARDWARE_INTERFACE_JOG_MODE

// System Headers
#include <string>
#include <vector>

// Kortex 3 API Headers
#include "SessionClientRpc.h"
#include "RouterMQTT.h"

#include "TransportClientUdp.h"
#include "RouterClient.h"
#include "SessionManager.h"

#include "BaseClientRpc.h"
#include "BaseCyclicClientRpc.h"

#include "Base.pb.h"

// ROS 2 Headers
#include "rclcpp/rclcpp.hpp"
// #include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

namespace k_api = Kinova::Api;

namespace kortex3_driver
{
class KortexHardwareInterfaceJogMode : public hardware_interface::SystemInterface
{
public:
  // Class Constructor
  KortexHardwareInterfaceJogMode();

  // Standard ros2_control lifecycle methods
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  // Methods to export hardware interfaces to the controller manager
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // Core methods for communication with the hardware
  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
  // --- Connection Parameters ---
  std::string robot_ip_;               ///< IP address of the robot controller.
  std::string username_;               ///< Username for session authentication.
  std::string password_;               ///< Password for session authentication.
  int mqtt_port_;                      ///< Port for low frequency MQTT communication (commands).
  int port_realtime_;                  ///< Port for realtime UDP communication (feedback and commands).
  int session_inactivity_timeout_;     ///< Inactivity period (in milliseconds) allowed before the session times out and closes on its own.
  int connection_inactivity_timeout_;  ///< Inactivity period (in milliseconds) allowed before the robot stops any movements initiated from this session.
  std::string gripper_name_;           ///< Which gripper is being used.
  uint16_t gripper_modbus_id_;         ///< The modbus ID to communicate with the gripper.


  // --- Kortex API Objects ---
  std::shared_ptr<k_api::RouterMQTT> router_mqtt_;
  std::shared_ptr<k_api::Session::SessionClient> session_mqtt_;

  std::unique_ptr<k_api::TransportClientUdp> transport_udp_;
  std::unique_ptr<k_api::RouterClient> router_udp_;
  std::unique_ptr<k_api::SessionManager> session_udp_;

  std::shared_ptr<k_api::Base::BaseClient> base_;
  std::shared_ptr<k_api::BaseCyclic::BaseCyclicClient> base_cyclic_;


  size_t actuator_count_;

  // --- State and Command Buffers ---
  k_api::BaseCyclic::Feedback feedback_;
  std::vector<double> joint_velocities_cmd_;  ///< Buffer for velocity commands (exported but unused in low-level mode).
  std::vector<double> joint_positions_;       ///< Buffer for joint position states.
  std::vector<double> joint_velocities_;      ///< Buffer for joint velocity states.
  std::vector<double> joint_torques_;         ///< Buffer for joint torque states.

  // Link6 operating mode and servoing mode
  k_api::Common::ModeSelection mode_selection_;
  k_api::Base::ServoingModeInformation servoing_mode_info_;
  k_api::Base::ServoingMode arm_mode_;

  // Fault management
  std::atomic<bool> in_fault_ = false;

  // --- Private Helper Methods ---
  void change_operating_mode(const k_api::Common::OperatingModeType& mode);
  void set_servoing_mode(const k_api::Base::ServoingMode& mode);
  void sendJointSpeedsCommand();
};

}  // namespace kortex3_driver

#endif  // KORTEX_HARDWARE_INTERFACE_JOG_MODE