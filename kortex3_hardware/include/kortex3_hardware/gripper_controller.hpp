/**
 * @file gripper_controller.hpp
 * @brief GripperController class for Robotiq finger gripper control via Modbus over MQTT.
 * @author Rafael Gomes Braga
 */

#pragma once

#ifndef _OS_UNIX
#define _OS_UNIX
#endif

#include <chrono>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "RouterMQTT.h"
#include "rclcpp/rclcpp.hpp"
#include "robotiq_gripper/Grippers/FingerGripper.h"
#include "robotiq_gripper/robotiq_plugin/GripperStatus.h"

namespace kortex3_driver
{
namespace k_api = Kinova::Api;

/**
 * @class MyFingerGripper
 * @brief Extends FingerGripper to provide a custom Modbus timeout.
 */
class MyFingerGripper : public FingerGripper
{
public:
  explicit MyFingerGripper(std::shared_ptr<slick::com::ModbusClientWrapper> wrapper)
  : FingerGripper(wrapper) {}

  uint32_t GetModbusTimeout() override { return 200; }
};

/**
 * @class GripperController
 * @brief Encapsulates state and Modbus operations for a single Robotiq finger gripper.
 *
 * All Modbus I/O is protected by an external mutex shared between all GripperControllers
 * on the same bus. Intended to be called from a dedicated background thread, not from
 * the ros2_control update loop.
 */
class GripperController
{
public:
  explicit GripperController(uint16_t default_modbus_id = 9)
    : modbus_id_(default_modbus_id) {}

  // Configuration
  std::string joint_name_;
  uint16_t modbus_id_;

  // Modbus objects
  std::shared_ptr<slick::com::ModbusClientWrapper> modbus_wrapper_;
  std::unique_ptr<MyFingerGripper> gripper_;
  bool initialized_ = false;

  // Internal state (owned by background thread)
  double position_ = 0.0;
  double velocity_ = 0.0;

  // Rate limiting
  std::chrono::steady_clock::time_point next_send_{};
  std::chrono::steady_clock::time_point next_poll_{};
  double last_cmd_pos_{std::numeric_limits<double>::quiet_NaN()};

  static constexpr std::chrono::milliseconds cmd_period_{50};    // 20 Hz
  static constexpr std::chrono::milliseconds poll_period_{100};  // 10 Hz

  /**
   * @brief Initialize and activate the gripper over Modbus.
   * @param router MQTT router used for Modbus communication (shared with arm session).
   * @param logger ROS logger.
   * @return true on success.
   */
  bool initialize(std::shared_ptr<k_api::RouterMQTT> router, const rclcpp::Logger & logger);

  /**
   * @brief Read the gripper position from Modbus. Rate-limited to poll_period_.
   *        Uses try_to_lock so it never blocks the caller.
   * @param mutex Shared Modbus bus mutex.
   * @param logger ROS logger.
   * @return Position in metres, or nullopt on failure.
   */
  std::optional<double> readPosition(std::mutex & mutex, const rclcpp::Logger & logger);

  /**
   * @brief Send a position command to the gripper. Rate-limited to cmd_period_.
   * @param position_metres Target position in metres [0, 0.025].
   * @param mutex Shared Modbus bus mutex.
   */
  void sendCommand(double position_metres, std::mutex & mutex);

  /**
   * @brief Gracefully close the Modbus connection and reset internal state.
   * @param mutex Shared Modbus bus mutex.
   */
  void shutdown(std::mutex & mutex);
};

}  // namespace kortex3_driver
