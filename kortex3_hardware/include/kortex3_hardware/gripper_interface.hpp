/**
 * @file gripper_interface.hpp
 * @brief Public facade for the (private) Robotiq gripper implementation.
 *
 * The concrete gripper logic (Robotiq/Modbus/slick) is proprietary and ships
 * only as a prebuilt shared library (libkortex3_private.so). This header is the
 * sole compile-time contract between the public ros2_control plugin and that
 * library: the public code holds a std::unique_ptr<IGripper> obtained from
 * makeFingerGripper() and never sees the private types/headers.
 *
 * KINOVA (R) KORTEX (TM), Link 6 Integration Team
 * Copyright (c) 2023 Kinova inc. All rights reserved.
 */

#ifndef KORTEX3_HARDWARE__GRIPPER_INTERFACE_HPP_
#define KORTEX3_HARDWARE__GRIPPER_INTERFACE_HPP_

#include <cstdint>
#include <memory>

// RouterMQTT is part of the public Kinova SDK (third_party/kortex3), not the
// private gripper code, so it is safe to reference here.
#include "RouterMQTT.h"

namespace kortex3_driver
{
namespace k_api = Kinova::Api;

namespace gripper
{

/**
 * @class IGripper
 * @brief Abstract interface to a single Robotiq finger gripper over Modbus.
 *
 * Method names/semantics mirror the private FingerGripper/ModbusClientWrapper
 * API so the public plugin's call sites are unchanged. The implementation lives
 * in the private shared library.
 */
class IGripper
{
public:
  virtual ~IGripper() = default;

  // --- Modbus connection ---
  /// Open the Modbus connection. Returns true on success (Ok).
  virtual bool TryInitConnection() = 0;
  /// Close the Modbus connection.
  virtual void CloseConnection() = 0;

  // --- Activation / status ---
  virtual void FreezeGripper() = 0;
  virtual bool ReadRegister() = 0;
  virtual void SetActivateRequest() = 0;
  virtual bool SendRequest() = 0;
  virtual bool GetActivateEcho() = 0;
  virtual int GetStatus() = 0;

  // --- Position command / feedback ---
  virtual std::uint8_t GetPosition() = 0;
  virtual void ClearGoToRequest() = 0;
  virtual void SetPositionRequest(std::uint8_t pos) = 0;
  virtual void SetSpeedRequest(std::uint8_t speed) = 0;
  virtual void SetGoToRequest() = 0;
};

/**
 * @brief Construct a Robotiq finger gripper bound to the given MQTT router.
 *
 * Creates the underlying Modbus wrapper and gripper (with the configured Modbus
 * timeout) but does not open the connection; call TryInitConnection() for that.
 *
 * @param router            Shared MQTT router (shared with the arm session).
 * @param modbus_id         Modbus slave id of the gripper.
 * @param modbus_timeout_ms Modbus response timeout in milliseconds.
 * @return Owning handle to the gripper, never null.
 */
std::unique_ptr<IGripper> makeFingerGripper(
  std::shared_ptr<k_api::RouterMQTT> router,
  std::uint16_t modbus_id,
  std::uint32_t modbus_timeout_ms = 200);

}  // namespace gripper
}  // namespace kortex3_driver

#endif  // KORTEX3_HARDWARE__GRIPPER_INTERFACE_HPP_
