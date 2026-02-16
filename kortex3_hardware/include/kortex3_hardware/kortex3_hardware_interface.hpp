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
#include "ProgramRunnerClientRpc.h"
#include "ProtectionZoneClientRpc.h"
#include "Base.pb.h"
#include "Common.pb.h"
#include "ProgramRunner.pb.h"
#include "ProgramConfig.pb.h"
#include "ProtectionZone.pb.h"
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
#include "kortex3_hardware/srv/simulate_estop.hpp"
#include "kortex3_hardware/srv/run_program.hpp"
#include "kortex3_hardware/srv/list_programs.hpp"
#include "kortex3_hardware/srv/stop_program.hpp"
#include "kortex3_hardware/srv/get_program_status.hpp"
#include "kortex3_hardware/srv/list_protection_zones.hpp"
#include "kortex3_hardware/msg/program_info.hpp"
#include "kortex3_hardware/msg/protection_zone_info.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "controller_manager_msgs/srv/list_controllers.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"

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

/**
 * @class GripperController
 * @brief Encapsulates state and operations for a single Robotiq gripper.
 */
class GripperController {
public:
  explicit GripperController(uint16_t default_modbus_id = 9)
    : modbus_id_(default_modbus_id) {}

  // Configuration
  std::string joint_name_;
  uint16_t modbus_id_;

  // Modbus communication
  std::shared_ptr<slick::com::ModbusClientWrapper> modbus_wrapper_;
  std::unique_ptr<MyFingerGripper> gripper_;
  bool initialized_ = false;

  // State
  double command_position_ = 0.0;
  double position_ = 0.0;
  double velocity_ = 0.0;

  // Timing for rate limiting
  std::chrono::steady_clock::time_point next_send_{};
  std::chrono::steady_clock::time_point next_poll_{};
  double last_cmd_pos_{std::numeric_limits<double>::quiet_NaN()};

  static constexpr std::chrono::milliseconds cmd_period_{50};   // 20 Hz
  static constexpr std::chrono::milliseconds poll_period_{100}; // 10 Hz

  /**
   * @brief Initialize and activate the gripper
   * @param router MQTT router for Modbus communication
   * @param logger ROS logger for messages
   * @return true if successful
   */
  bool initialize(std::shared_ptr<k_api::RouterMQTT> router, const rclcpp::Logger& logger);

  /**
   * @brief Read gripper position from Modbus
   * @param mutex Mutex protecting Modbus access (shared between grippers)
   * @param logger ROS logger for messages
   * @return Optional position value in radians
   */
  std::optional<double> readPosition(std::mutex& mutex, const rclcpp::Logger& logger);

  /**
   * @brief Send position command to gripper via Modbus
   * @param position_radians Desired position in radians
   * @param mutex Mutex protecting Modbus access (shared between grippers)
   */
  void sendCommand(double position_radians, std::mutex& mutex);

  /**
   * @brief Close Modbus connection and reset gripper
   * @param mutex Mutex protecting Modbus access (shared between grippers)
   */
  void shutdown(std::mutex& mutex);
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

private:
  // --- Private Helper Methods ---
  void check_and_power_on_robot();
  void send_zero_velocities();
  void change_operating_mode(const k_api::Common::OperatingModeType& mode);
  // Controller management helpers for fault recovery
  std::vector<std::string> get_active_motion_controllers();
  // --- ROS Service Handlers ---
  void handle_set_operating_mode(
      const std::shared_ptr<kortex3_hardware::srv::SetOperatingMode::Request> request,
      std::shared_ptr<kortex3_hardware::srv::SetOperatingMode::Response> response);
  void handle_clear_faults(
      const std::shared_ptr<kortex3_hardware::srv::ClearFaults::Request> request,
      std::shared_ptr<kortex3_hardware::srv::ClearFaults::Response> response);
  void handle_simulate_estop(
      const std::shared_ptr<kortex3_hardware::srv::SimulateEstop::Request> request,
      std::shared_ptr<kortex3_hardware::srv::SimulateEstop::Response> response);
  void handle_run_program(
      const std::shared_ptr<kortex3_hardware::srv::RunProgram::Request> request,
      std::shared_ptr<kortex3_hardware::srv::RunProgram::Response> response);
  void handle_list_programs(
      const std::shared_ptr<kortex3_hardware::srv::ListPrograms::Request> request,
      std::shared_ptr<kortex3_hardware::srv::ListPrograms::Response> response);
  void handle_stop_program(
      const std::shared_ptr<kortex3_hardware::srv::StopProgram::Request> request,
      std::shared_ptr<kortex3_hardware::srv::StopProgram::Response> response);
  void handle_get_program_status(
      const std::shared_ptr<kortex3_hardware::srv::GetProgramStatus::Request> request,
      std::shared_ptr<kortex3_hardware::srv::GetProgramStatus::Response> response);
  void handle_list_protection_zones(
      const std::shared_ptr<kortex3_hardware::srv::ListProtectionZones::Request> request,
      std::shared_ptr<kortex3_hardware::srv::ListProtectionZones::Response> response);

  // Helper function to check if a program status represents an active program
  bool is_program_active(k_api::ProgramRunner::Status status) const;

  // Helper function to check if unsafe controllers are active
  bool check_unsafe_controllers_active(std::string& error_message);

  // --- Connection Parameters ---
  std::string robot_ip_;       ///< IP address of the robot controller.
  std::string username_;       ///< Username for session authentication.
  std::string password_;       ///< Password for session authentication.
  std::string gripper_name_;   ///< Which gripper is being used
  uint16_t gripper_modbus_id_; ///< The modbus ID to communicate with the gripper
  int mqtt_port_;              ///< TCP port for MQTT communication (commands).
  int udp_feedback_port_;      ///< UDP port for high-frequency feedback.

  // --- Kortex API Objects ---
  std::shared_ptr<k_api::RouterMQTT> router_mqtt_;
  std::shared_ptr<k_api::Session::SessionClient> session_mqtt_;
  std::shared_ptr<k_api::Base::BaseClient> base_mqtt_;
  std::shared_ptr<k_api::ProgramRunner::ProgramRunnerClient> program_runner_;
  std::shared_ptr<k_api::ProtectionZone::ProtectionZoneClient> protection_zone_;
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

  // Gripper controllers
  bool use_internal_bus_gripper_comm_;
  GripperController gripper_a_;
  GripperController gripper_b_;
  std::mutex gripper_mtx_;  ///< Shared mutex for all Modbus operations



  // --- ROS 2 Components ---
  rclcpp::Node::SharedPtr node_ptr_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_publisher_;
  rclcpp::Service<kortex3_hardware::srv::SetOperatingMode>::SharedPtr set_operating_mode_service_;
  rclcpp::Service<kortex3_hardware::srv::ClearFaults>::SharedPtr clear_faults_service_;
  rclcpp::Service<kortex3_hardware::srv::SimulateEstop>::SharedPtr simulate_estop_service_;
  rclcpp::Service<kortex3_hardware::srv::RunProgram>::SharedPtr run_program_service_;
  rclcpp::Service<kortex3_hardware::srv::ListPrograms>::SharedPtr list_programs_service_;
  rclcpp::Service<kortex3_hardware::srv::StopProgram>::SharedPtr stop_program_service_;
  rclcpp::Service<kortex3_hardware::srv::GetProgramStatus>::SharedPtr get_program_status_service_;
  rclcpp::Service<kortex3_hardware::srv::ListProtectionZones>::SharedPtr list_protection_zones_service_;

  // --- Internal State Flags ---
  Kinova::Api::Common::ArmState           last_arm_state_{Kinova::Api::Common::ARMSTATE_UNSPECIFIED};
  Kinova::Api::Common::OperatingModeType  last_operating_mode_{Kinova::Api::Common::OPERATING_MODE_UNSPECIFIED};
  bool                                    in_fault_{false}; ///< Flag to indicate if the robot is in a fault state.
  bool                                    pending_controller_deactivation_{false}; ///< Flag to trigger controller deactivation from read()
  std::vector<std::string>                controllers_to_deactivate_; ///< Controllers to deactivate after recovery
  int                                     deactivation_verify_countdown_{0}; ///< Cycles to wait before verifying deactivation
  bool                                    fault_reported_{false};
  bool                                    fault_recently_logged_{false}; ///< Rate limiting flag for fault logging
  
  // Program runner state tracking
  Kinova::Api::ProgramRunner::Status last_program_status_{Kinova::Api::ProgramRunner::STATUS_IDLE};
  std::chrono::steady_clock::time_point program_end_time_{};
  std::chrono::steady_clock::time_point   last_fault_log_time_{}; ///< Last time fault was logged

  ///< Static logger for the class.
  static const rclcpp::Logger LOGGER;
};

} // namespace kortex3_driver
