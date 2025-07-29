/**
 * @file test_read_calibration.cpp
 * @brief Contains a calibration URDF testing for the ros2_control hardware interface for the Kinova Link6 robot arm.
 * @author Anas Houssaini
 */

#include <iostream>
#include <memory>
#include <signal.h>
#include <atomic>

#include "kortex3_hardware/kortex3_hardware_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"


std::atomic<bool> running(true);
void signal_handler(int sig) { if (sig == SIGINT || sig == SIGTERM) running = false; }


constexpr const char* ROBOT_IP  = "192.168.1.10";
constexpr uint16_t    MQTT_PORT = 1883;
constexpr const char* USERNAME  = "admin";
constexpr const char* PASSWORD  = "admin";

int main(int argc, char** argv)
{
  signal(SIGINT,  signal_handler);
  signal(SIGTERM, signal_handler);

  rclcpp::init(argc, argv);


  kortex3_driver::Kortex3HardwareInterface hw;
  rclcpp_lifecycle::State lc_state;

  hardware_interface::HardwareInfo hw_info;
  hw_info.name = "Kortex3Robot";
  hw_info.type = "system";
  hw_info.hardware_parameters["robot_ip"]          = ROBOT_IP;
  hw_info.hardware_parameters["username"]          = USERNAME;
  hw_info.hardware_parameters["password"]          = PASSWORD;
  hw_info.hardware_parameters["mqtt_port"]         = std::to_string(MQTT_PORT);
  hw_info.hardware_parameters["udp_feedback_port"] = "10001";

  for (int i = 1; i <= 6; ++i) {
    hardware_interface::ComponentInfo joint;
    joint.name = "joint_" + std::to_string(i);
    hardware_interface::InterfaceInfo cmd{hardware_interface::HW_IF_VELOCITY};
    joint.command_interfaces.push_back(cmd);
    for (auto st : {hardware_interface::HW_IF_POSITION,
                    hardware_interface::HW_IF_VELOCITY,
                    hardware_interface::HW_IF_EFFORT})
      joint.state_interfaces.emplace_back(hardware_interface::InterfaceInfo{st});
    hw_info.joints.push_back(joint);
  }

  if (hw.on_init(hw_info)            != hardware_interface::CallbackReturn::SUCCESS ||
      hw.on_configure(lc_state)      != hardware_interface::CallbackReturn::SUCCESS ||
      hw.on_activate(lc_state)       != hardware_interface::CallbackReturn::SUCCESS)
  {
    std::cerr << "✗ Failed to initialise Kortex3HardwareInterface\n";
    rclcpp::shutdown();
    return 1;
  }
  std::cout << "✓ Interface activated\n";


  if (hw.dump_calibration("C61"))
    std::cout << "✓ Bundle written to link6_description/calibration\n";
  else
    std::cerr << "✗ Calibration dump failed\n";


  hw.on_deactivate(lc_state);
  rclcpp::shutdown();
  std::cout << "=== Test complete ===\n";
  return 0;
}
