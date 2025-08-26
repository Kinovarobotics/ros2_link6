/**
 * @file kortex3_hardware_interface.hpp
 * @brief Contains the ros2_control hardware interface for the Kinova Link6 robot arm.
 * @author Anas Houssaini
 */

#pragma once

// For cross-platform compatibility.
#ifndef _OS_UNIX
#define _OS_UNIX
#endif

// System Headers
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <fstream>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <atomic>
#include <mutex>
#include <filesystem>

// Kortex 3 API Headers
#include "RouterMQTT.h"
#include "RouterClient.h"
#include "SessionManager.h"
#include "SessionClientRpc.h"
#include "BaseClientRpc.h"
#include "BaseCyclicClientRpc.h"
#include "Base.pb.h"
#include "Common.pb.h"
#include "TransportClientUdp.h"

// ROS 2 Headers
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/publisher.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "kortex3_hardware/srv/set_operating_mode.hpp"
#include "kortex3_hardware/srv/clear_faults.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include <ament_index_cpp/get_package_share_directory.hpp>

// Robotiq gripper plugin headers
#include "robotiq_gripper/Grippers/FingerGripper.h"
#include "robotiq_gripper/robotiq_plugin/GripperStatus.h"

namespace kortex3_driver
{
namespace k_api = Kinova::Api;
namespace fs = std::filesystem;

/**
 * @class Kortex3HardwareInterface
 * @brief A ros2_control SystemInterface for the Kinova Gen3 robot.
 *
 * This class implements the necessary methods to interface with the Kinova Gen3 arm,
 * enabling control via the ros2_control framework. It handles communication for
 * low-frequency commands (MQTT) and high-frequency state feedback (UDP).
 */

class MyFingerGripper : public FingerGripper {
public:
  MyFingerGripper(std::shared_ptr<slick::com::ModbusClientWrapper> wrapper)
  : FingerGripper(wrapper) {}

  uint32_t GetModbusTimeout() override {
    return 200; // Or retrieve from wrapper if needed
  }
};

class Kortex3HardwareInterface : public hardware_interface::SystemInterface
{
public:
  /** @brief Constructor for Kortex3HardwareInterface. */
  Kortex3HardwareInterface();

  /** @brief Default destructor. */
  virtual ~Kortex3HardwareInterface() = default;

  // Standard ros2_control lifecycle methods
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  // Methods to export hardware interfaces to the controller manager
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // Core methods for communication with the hardware
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // --- Utility and Debugging Methods ---
  /** @brief Gets the current joint positions. For debugging. */
  const std::vector<double>& get_joint_positions() const { return joint_positions_; }
  /** @brief Gets the current joint velocities. For debugging. */
  const std::vector<double>& get_joint_velocities() const { return joint_velocities_; }
  /** @brief Gets the current joint torques. For debugging. */
  const std::vector<double>& get_joint_torques() const { return joint_torques_; }

  /**
   * @brief Fetches the robot's calibration data and saves it to a file.
   * @param dst The destination file path.
   * @return True on success, false otherwise.
   */
  bool dump_calibration(const std::string& dst);

  /**
   * @brief Triggers the full robot calibration process, generating a calibrated URDF.
   * @return True on success, false otherwise.
   */
  bool calibrate_robot();

private:
  std::mutex gripper_mtx_;
  // --- Private Helper Methods ---
  void check_and_power_on_robot();
  void send_zero_velocities();
  void change_operating_mode(const k_api::Common::OperatingModeType& mode);

  // --- ROS Service Handlers ---
  void handle_set_operating_mode(
      const std::shared_ptr<kortex3_hardware::srv::SetOperatingMode::Request> request,
      std::shared_ptr<kortex3_hardware::srv::SetOperatingMode::Response> response);
  void handle_clear_faults(
      const std::shared_ptr<kortex3_hardware::srv::ClearFaults::Request> request,
      std::shared_ptr<kortex3_hardware::srv::ClearFaults::Response> response);
  std::optional<double> readGripperPosition();
  void sendGripperCommand(double position_radians);

  // --- Connection Parameters ---
  std::string robot_ip_;      ///< IP address of the robot controller.
  std::string username_;      ///< Username for session authentication.
  std::string password_;      ///< Password for session authentication.
  int mqtt_port_;             ///< TCP port for MQTT communication (commands).
  int udp_feedback_port_;     ///< UDP port for high-frequency feedback.

  // --- Kortex API Objects ---
  std::shared_ptr<k_api::RouterMQTT> router_mqtt_;
  std::shared_ptr<k_api::Session::SessionClient> session_mqtt_;
  std::shared_ptr<k_api::Base::BaseClient> base_mqtt_;
  std::unique_ptr<k_api::TransportClientUdp> transport_udp_feedback_;
  std::unique_ptr<k_api::RouterClient> router_udp_feedback_;
  std::unique_ptr<k_api::SessionManager> session_udp_;
  std::shared_ptr<k_api::BaseCyclic::BaseCyclicClient> base_cyclic_udp_;

  // --- State and Command Buffers ---
  size_t actuator_count_;
  std::vector<double> joint_velocities_cmd_; ///< Buffer for velocity commands from ros2_control.
  std::vector<double> joint_positions_;      ///< Buffer for joint position states.
  std::vector<double> joint_velocities_;     ///< Buffer for joint velocity states.
  std::vector<double> joint_torques_;        ///< Buffer for joint torque states.
  // Gripper
  std::shared_ptr<slick::com::ModbusClientWrapper> modbus_wrapper_;
  std::unique_ptr<MyFingerGripper> gripper_;
  std::string gripper_joint_name_;
  double gripper_command_position_ = 0.0;
  double gripper_position_ = 0.0;
  double gripper_velocity_ = 0.0;
  bool gripper_initialized_ = false;
  // Timing
  // Gripper send gating (tune periods as you like) for write
  std::chrono::steady_clock::time_point next_gripper_send_{};
  double last_gripper_cmd_pos_{std::numeric_limits<double>::quiet_NaN()};
  const std::chrono::milliseconds gripper_cmd_period_{50};   // 20 Hz send budget
  // Poll gating (avoid spamming Modbus) for read
  std::chrono::steady_clock::time_point next_gripper_poll_{};
  const std::chrono::milliseconds gripper_poll_period_{100};  // 10 Hz



  // --- ROS 2 Components ---
  rclcpp::Node::SharedPtr node_ptr_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_publisher_;
  rclcpp::Service<kortex3_hardware::srv::SetOperatingMode>::SharedPtr set_operating_mode_service_;
  rclcpp::Service<kortex3_hardware::srv::ClearFaults>::SharedPtr clear_faults_service_;

  // --- Internal State Flags ---
  Kinova::Api::Common::ArmState           last_arm_state_{Kinova::Api::Common::ARMSTATE_UNSPECIFIED};
  Kinova::Api::Common::OperatingModeType  last_operating_mode_{Kinova::Api::Common::OPERATING_MODE_UNSPECIFIED};
  bool                                    in_fault_{false}; ///< Flag to indicate if the robot is in a fault state.
  bool                                    fault_reported_{false};

  ///< Static logger for the class.
  static const rclcpp::Logger LOGGER;
};

} // namespace kortex3_driver