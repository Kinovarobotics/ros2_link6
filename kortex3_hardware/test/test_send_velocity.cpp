/**
* @file test_read_only.cpp
 * @brief Implements send velocity commands using Kortex3HardwareInterface for the Kinova Link6 robot arm.
 * @author Anas Houssaini
 */

#include <iostream>
#include <memory>
#include <thread>
#include <chrono>
#include <signal.h>
#include <iomanip>
#include <cmath>
#include <vector>

#include "kortex3_hardware/kortex3_hardware_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include "rclcpp/rclcpp.hpp"

std::atomic<bool> running(true);

void signal_handler(int signal)
{
  if (signal == SIGINT || signal == SIGTERM)
  {

    running = false;
  }
}


double deg_to_rad(double degrees)
{
  return degrees * M_PI / 180.0;
}


double rad_to_deg(double radians)
{
  return radians * 180.0 / M_PI;
}

int main(int argc, char** argv)
{
  signal(SIGINT, signal_handler);
  signal(SIGTERM, signal_handler);


  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("kortex3_velocity_test");
  std::unique_ptr<kortex3_driver::Kortex3HardwareInterface> hw = nullptr;
  rclcpp_lifecycle::State state;

  try
  {
    std::cout << "=== Kortex3 Hardware Interface - Velocity Control Test ===" << std::endl;
    hw = std::make_unique<kortex3_driver::Kortex3HardwareInterface>();


    hardware_interface::HardwareInfo hw_info;
    hw_info.name = "Kortex3Robot";
    hw_info.type = "system";


    hw_info.hardware_parameters["robot_ip"] = "192.168.1.10";
    hw_info.hardware_parameters["username"] = "admin";
    hw_info.hardware_parameters["password"] = "admin";
    hw_info.hardware_parameters["mqtt_port"] = "1883";
    hw_info.hardware_parameters["udp_feedback_port"] = "10001";

<<<<<<< HEAD

=======
>>>>>>> 4c628d0 (post-testing modifications)
    // Define arm joints
    for (int i = 1; i <= 6; i++)
    {
      hardware_interface::ComponentInfo joint;
      joint.name = "joint_" + std::to_string(i);
      hardware_interface::InterfaceInfo cmd_iface;
      cmd_iface.name = hardware_interface::HW_IF_VELOCITY;
      joint.command_interfaces.push_back(cmd_iface);
      hardware_interface::InterfaceInfo pos_iface;
      pos_iface.name = hardware_interface::HW_IF_POSITION;
      joint.state_interfaces.push_back(pos_iface);
      hardware_interface::InterfaceInfo vel_iface;
      vel_iface.name = hardware_interface::HW_IF_VELOCITY;
      joint.state_interfaces.push_back(vel_iface);
      hardware_interface::InterfaceInfo eff_iface;
      eff_iface.name = hardware_interface::HW_IF_EFFORT;
      joint.state_interfaces.push_back(eff_iface);
      hw_info.joints.push_back(joint);
    }
    
    // Add gripper joint (7th joint) - matches URDF configuration
    {
      hardware_interface::ComponentInfo gripper_joint;
      gripper_joint.name = "robotiq_85_left_knuckle_joint";

      // Command interface (position for gripper)
      hardware_interface::InterfaceInfo cmd_iface;
      cmd_iface.name = hardware_interface::HW_IF_POSITION;
      gripper_joint.command_interfaces.push_back(cmd_iface);

      // State interfaces (position and velocity)
      hardware_interface::InterfaceInfo pos_iface;
      pos_iface.name = hardware_interface::HW_IF_POSITION;
      gripper_joint.state_interfaces.push_back(pos_iface);

      hardware_interface::InterfaceInfo vel_iface;
      vel_iface.name = hardware_interface::HW_IF_VELOCITY;
      gripper_joint.state_interfaces.push_back(vel_iface);

      hw_info.joints.push_back(gripper_joint);
    }


    std::cout << "\n1. Initializing hardware interface..." << std::endl;
    if (hw->on_init(hw_info) != hardware_interface::CallbackReturn::SUCCESS)
    {
      throw std::runtime_error("Failed to initialize hardware interface");
    }
    std::cout << "   ✓ Initialized" << std::endl;


    std::cout << "\n2. Configuring hardware interface..." << std::endl;
    if (hw->on_configure(state) != hardware_interface::CallbackReturn::SUCCESS)
    {
      throw std::runtime_error("Failed to configure hardware interface");
    }
    std::cout << "   ✓ Configured" << std::endl;


    std::cout << "\n3. Exporting interfaces..." << std::endl;
    auto state_interfaces = hw->export_state_interfaces();
    auto command_interfaces = hw->export_command_interfaces();
    std::cout << "   ✓ Exported " << state_interfaces.size() << " state interfaces" << std::endl;
    std::cout << "   ✓ Exported " << command_interfaces.size() << " command interfaces" << std::endl;


    std::cout << "\n4. Activating (connecting to robot)..." << std::endl;
    if (hw->on_activate(state) != hardware_interface::CallbackReturn::SUCCESS)
    {
      throw std::runtime_error("Failed to activate hardware interface");
    }
    std::cout << "   ✓ Connected to robot!" << std::endl;

    std::cout << "\n5. Select velocity test mode:" << std::endl;
    std::cout << "   0: Zero velocities (stop all joints)" << std::endl;
    std::cout << "   1: Small sine wave (3 deg/s amplitude, 0.5 Hz)" << std::endl;
    std::cout << "   2: Constant velocity on joint 1 (5 deg/s)" << std::endl;
    std::cout << "   3: Custom velocities (you specify)" << std::endl;
    std::cout << "   4: Constant velocity on last joint (5 deg/s)" << std::endl;
    std::cout << "   5: Strong sine wave (20 deg/s, 0.5 Hz)" << std::endl; // <-- ADD THIS LINE
    std::cout << "\nEnter mode (0-5): ";

    int mode = 0;
    std::cin >> mode;

    std::vector<double> custom_velocities_deg(6, 0.0);
    if (mode == 3)
    {
      std::cout << "\nEnter velocity for each joint in deg/s:" << std::endl;
      for (int i = 0; i < 6; i++)
      {
        std::cout << "Joint " << (i+1) << " velocity [deg/s]: ";
        std::cin >> custom_velocities_deg[i];
      }
    }

    std::cout << "\n6. Starting control loop (press Ctrl+C to stop)..." << std::endl;
    std::cout << "   Sending velocity commands at 100Hz" << std::endl;

    int loop_count = 0;
    auto start_time = std::chrono::steady_clock::now();

    while (running && rclcpp::ok())
    {
      auto loop_start = std::chrono::steady_clock::now();
      if (hw->read(node->now(), rclcpp::Duration(0, 10000000)) != hardware_interface::return_type::OK) {
          std::cerr << "Read failed!" << std::endl;
          break;
      }
      double t = std::chrono::duration<double>(loop_start - start_time).count();
      for (size_t i = 0; i < command_interfaces.size(); i++) {
        double velocity_deg = 0.0;
        switch (mode) {
          case 0: velocity_deg = 0.0; break;
          case 1: velocity_deg = 3.0 * std::sin(2 * M_PI * 0.5 * t); break;
          case 2: velocity_deg = (i == 0) ? 5.0 : 0.0; break;
          case 3: velocity_deg = custom_velocities_deg[i]; break;
          case 4: velocity_deg = (i == 5) ? 5.0 : 0.0; break;
          case 5: velocity_deg = 20.0 * std::sin(2 * M_PI * 0.5 * t); break; // <-- ADD THIS LINE
        }
        command_interfaces[i].set_value(deg_to_rad(velocity_deg));
      }

      if (hw->write(node->now(), rclcpp::Duration(0, 10000000)) != hardware_interface::return_type::OK) {
          std::cerr << "Write failed!" << std::endl;
          break;
      }

      if (loop_count % 10 == 0) {
        std::cout << "\033[2J\033[H";
        std::cout << "=== Kortex3 Velocity Control Test ===" << std::endl;
        std::cout << "Mode: ";
        switch(mode) {
          case 0: std::cout << "Zero velocities"; break;
          case 1: std::cout << "Sine wave (3 deg/s, 0.5 Hz)"; break;
          case 2: std::cout << "Joint 1 constant (5 deg/s)"; break;
          case 3: std::cout << "Custom velocities"; break;
          case 4: std::cout << "Joint 6 constant (5 deg/s)"; break;
          case 5: std::cout << "Strong sine wave (20 deg/s, 0.5 Hz)"; break; // <-- ADD THIS LINE
        }
        std::cout << std::endl;
        std::cout << "Time: " << std::fixed << std::setprecision(1) << t << "s" << std::endl;
        std::cout << "\nJoint States:" << std::endl;
        std::cout << "----------------------------------------------------------------------------" << std::endl;
        std::cout << "Joint | Pos [deg] | Vel [deg/s] | Torque [Nm] | Cmd Vel [deg/s]" << std::endl;
        std::cout << "------|-----------|-------------|-------------|----------------" << std::endl;
        const auto& positions = hw->get_joint_positions();
        const auto& velocities = hw->get_joint_velocities();
        const auto& torques = hw->get_joint_torques();
        if (positions.size() >= 6 && velocities.size() >= 6 && torques.size() >= 6) {
            for (size_t i = 0; i < 6; i++) {
                std::cout << "  " << i + 1 << "   | "
                          << std::setw(9) << std::fixed << std::setprecision(2) << rad_to_deg(positions[i]) << " | "
                          << std::setw(11) << std::fixed << std::setprecision(3) << rad_to_deg(velocities[i]) << " | "
                          << std::setw(11) << std::fixed << std::setprecision(3) << torques[i] << " | "
                          << std::setw(14) << std::fixed << std::setprecision(3) << rad_to_deg(command_interfaces[i].get_value()) << std::endl;
            }
        }
        std::cout << "\nPress Ctrl+C to stop..." << std::endl;
      }
      loop_count++;
      std::this_thread::sleep_until(loop_start + std::chrono::milliseconds(10));
    }
  }
  catch (const std::exception& e)
  {
    std::cerr << "Exception: " << e.what() << std::endl;
    if (hw) {
      // If an exception occurs, still try to deactivate cleanly
      hw->on_deactivate(state);
    }
    rclcpp::shutdown();
    return 1;
  }

  // This block runs after Ctrl+C is pressed or the loop exits
  if (hw)
  {
    std::cout << "\nLoop terminated. Stopping robot (sending zero velocities)..." << std::endl;
    // We need to get the command interfaces again or have them in scope
    // For simplicity, we create a zero velocity vector and use a helper on the hw iface
    // Or, we can just call the public send_zero_velocities() if we make it public
    // Let's modify the hardware interface to have a public stop function

    // Simplest change: Re-export command interfaces to stop the robot
    auto command_interfaces = hw->export_command_interfaces();
    for (auto& cmd_iface : command_interfaces)
    {
      cmd_iface.set_value(0.0);
    }
    hw->write(node->now(), rclcpp::Duration(0, 10000000));
    std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Give it a moment to send
    std::cout << "   ✓ Robot stopped." << std::endl;

  }

  std::cout << "\n=== Test Complete ===" << std::endl;
  rclcpp::shutdown();
  return 0;
}