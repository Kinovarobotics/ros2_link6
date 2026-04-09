/**
 * @file gripper_controller.cpp
 * @brief Implementation of GripperController for Robotiq finger grippers.
 * @author Rafael Gomes Braga
 */

#include "kortex3_hardware/gripper_controller.hpp"

#include <algorithm>
#include <thread>

namespace kortex3_driver
{

bool GripperController::initialize(
  std::shared_ptr<k_api::RouterMQTT> router, const rclcpp::Logger & logger)
{
  RCLCPP_INFO(logger, "Initializing gripper '%s' with Modbus ID %u",
              joint_name_.c_str(), modbus_id_);

  modbus_wrapper_ = std::make_shared<slick::com::ModbusClientWrapper>(router, modbus_id_);

  if (modbus_wrapper_->TryInitConnection() != slick::com::ModbusError::Ok) {
    RCLCPP_ERROR(logger, "Failed to connect to gripper '%s' via Modbus.", joint_name_.c_str());
    return false;
  }
  RCLCPP_INFO(logger, "Gripper '%s' Modbus connection established.", joint_name_.c_str());

  gripper_ = std::make_unique<MyFingerGripper>(modbus_wrapper_);
  initialized_ = true;

  RCLCPP_INFO(logger, "Activating gripper '%s'...", joint_name_.c_str());
  gripper_->FreezeGripper();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  gripper_->ReadRegister();

  gripper_->SetActivateRequest();
  gripper_->SendRequest();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  gripper_->ReadRegister();

  const auto timeout = std::chrono::seconds(10);
  const auto start   = std::chrono::steady_clock::now();
  bool activated = false;

  while (std::chrono::steady_clock::now() - start < timeout) {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    if (!gripper_->ReadRegister()) {
      RCLCPP_WARN(logger, "Failed to read gripper '%s' status during activation.",
                  joint_name_.c_str());
      continue;
    }
    if (gripper_->GetActivateEcho()) {
      activated = true;
      RCLCPP_INFO(logger, "Gripper '%s' activated.", joint_name_.c_str());
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      break;
    }
    RCLCPP_DEBUG(logger, "Gripper '%s' activating... (STA=%d)",
                 joint_name_.c_str(), gripper_->GetStatus());
  }

  if (!activated) {
    RCLCPP_ERROR(logger, "Gripper '%s' activation timed out.", joint_name_.c_str());
    return false;
  }
  return true;
}

std::optional<double> GripperController::readPosition(
  std::mutex & mutex, const rclcpp::Logger & logger)
{
  const auto now = std::chrono::steady_clock::now();
  if (now < next_poll_) {
    return position_;  // return cached value without touching Modbus
  }

  std::unique_lock<std::mutex> lk(mutex, std::try_to_lock);
  if (!lk.owns_lock()) {
    // Another Modbus op in-flight; back off briefly
    next_poll_ = now + std::chrono::milliseconds(10);
    return position_;
  }

  if (!initialized_) return std::nullopt;

  bool ok = gripper_->ReadRegister();
  next_poll_ = now + poll_period_;
  if (!ok) return std::nullopt;

  const uint8_t raw = gripper_->GetPosition();
  position_ = 0.025 - (static_cast<double>(raw) / 255.0 * 0.025);
  return position_;
}

void GripperController::sendCommand(double position_metres, std::mutex & mutex)
{
  const auto now = std::chrono::steady_clock::now();
  if (now < next_send_) return;

  const double clamped = std::clamp(position_metres, 0.0, 0.025);
  if (last_cmd_pos_ == last_cmd_pos_ &&  // not NaN
      std::abs(clamped - last_cmd_pos_) < 0.0001) {
    return;  // change too small to bother
  }

  std::lock_guard<std::mutex> lk(mutex);
  if (!initialized_ || !gripper_) return;

  const uint8_t pos = static_cast<uint8_t>((1.0 - (clamped / 0.025)) * 255.0);
  const uint8_t spd = 255;

  gripper_->ClearGoToRequest();
  (void)gripper_->SendRequest();
  gripper_->SetPositionRequest(pos);
  gripper_->SetSpeedRequest(spd);
  gripper_->SetGoToRequest();
  (void)gripper_->SendRequest();

  last_cmd_pos_ = clamped;
  next_send_ = now + cmd_period_;
}

void GripperController::shutdown(std::mutex & mutex)
{
  std::lock_guard<std::mutex> lk(mutex);
  if (modbus_wrapper_) modbus_wrapper_->CloseConnection();
  gripper_.reset();
  modbus_wrapper_.reset();
  initialized_ = false;
}

}  // namespace kortex3_driver
