/**
 * @file test_read_only.cpp
 * @brief Implements a read robot state using Kortex3HardwareInterface for the Kinova Link6 robot arm.
 * @author Anas Houssaini
 */

#include <iostream>
#include <memory>
#include <thread>
#include <chrono>
#include <signal.h>
#include <iomanip>
#include <mutex> // For protecting shared force/torque data

#ifndef _OS_UNIX
# define _OS_UNIX
#endif
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>

#include "TransportClientUdp.h" // Needed for the hardware interface itself
#include "RouterClient.h"       // Needed for the hardware interface itself

#include "kortex3_hardware/kortex3_hardware_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp" // For subscribing to wrench data

std::atomic<bool> running(true);

// Global variables to store the latest force/torque data
// Protected by a mutex for thread safety
std::mutex g_wrench_mutex;
geometry_msgs::msg::WrenchStamped g_latest_wrench;

void signal_handler(int signal)
{
  if (signal == SIGINT || SIGTERM)
  {
    std::cout << "\nShutting down..." << std::endl;
    running = false;
  }
}

// Callback function for the WrenchStamped subscriber
void wrench_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(g_wrench_mutex);
  g_latest_wrench = *msg;
}

int main(int argc, char** argv)
{
  signal(SIGINT, signal_handler);
  signal(SIGTERM, signal_handler);

  // Initialize ROS 2
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("kortex3_read_test");

  // Create a subscriber for the wrench topic
  auto wrench_subscriber = node->create_subscription<geometry_msgs::msg::WrenchStamped>(
      "kortex/tool_wrench",
      10, // QoS history depth
      wrench_callback
  );
  std::cout << "Subscribed to kortex/tool_wrench topic." << std::endl;

  try
  {
    std::cout << "=== Kortex3 Hardware Interface - Read Only Test ===" << std::endl;

    // Create hardware interface
    auto hw = std::make_unique<kortex3_driver::Kortex3HardwareInterface>();

    // Setup hardware info
    hardware_interface::HardwareInfo hw_info;
    hw_info.name = "Kortex3Robot";
    hw_info.type = "system";

    // Set connection parameters
    hw_info.hardware_parameters["robot_ip"] = "192.168.1.10";
    hw_info.hardware_parameters["username"] = "admin";
    hw_info.hardware_parameters["password"] = "admin";
    hw_info.hardware_parameters["mqtt_port"] = "1883";
    hw_info.hardware_parameters["udp_feedback_port"] = "10001";

    // Define joints
    for (int i = 1; i <= 6; i++)
    {
      hardware_interface::ComponentInfo joint;
      joint.name = "joint_" + std::to_string(i);

      // Command interface (velocity only)
      hardware_interface::InterfaceInfo cmd_iface;
      cmd_iface.name = hardware_interface::HW_IF_VELOCITY;
      joint.command_interfaces.push_back(cmd_iface);

      // State interfaces
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

    // Initialize
    std::cout << "\n1. Initializing hardware interface..." << std::endl;
    if (hw->on_init(hw_info) != hardware_interface::CallbackReturn::SUCCESS)
    {
      throw std::runtime_error("Failed to initialize hardware interface");
    }
    std::cout << "   ✓ Initialized" << std::endl;

    // Configure
    std::cout << "\n2. Configuring hardware interface..." << std::endl;
    rclcpp_lifecycle::State state;
    if (hw->on_configure(state) != hardware_interface::CallbackReturn::SUCCESS)
    {
      throw std::runtime_error("Failed to configure hardware interface");
    }
    std::cout << "   ✓ Configured" << std::endl;

    // Export interfaces (still necessary for ros2_control compliance)
    std::cout << "\n3. Exporting interfaces..." << std::endl;
    auto state_interfaces = hw->export_state_interfaces();
    auto command_interfaces = hw->export_command_interfaces();
    std::cout << "   ✓ Exported " << state_interfaces.size() << " state interfaces" << std::endl;
    std::cout << "   ✓ Exported " << command_interfaces.size() << " command interfaces" << std::endl;

    // Activate (connect to robot)
    std::cout << "\n4. Activating (connecting to robot)..." << std::endl;
    if (hw->on_activate(state) != hardware_interface::CallbackReturn::SUCCESS)
    {
      throw std::runtime_error("Failed to activate hardware interface");
    }
    std::cout << "   ✓ Connected to robot!" << std::endl;

    // Read loop
    std::cout << "\n5. Starting read loop (press Ctrl+C to stop)..." << std::endl;
    std::cout << "   Reading joint states at 100Hz" << std::endl;
    std::cout << "   NOTE: NOT sending any commands - read only mode\n" << std::endl;

    int loop_count = 0;
    auto start_time = std::chrono::steady_clock::now();

    while (running && rclcpp::ok())
    {
      auto loop_start = std::chrono::steady_clock::now();

      // Spin the node to process the subscription callbacks (WrenchStamped)
      rclcpp::spin_some(node); // This is essential for the subscriber callback to be called

      // Read state from the robot into the hardware interface's internal buffers
      if (hw->read(node->now(), rclcpp::Duration(0, 10000000)) != hardware_interface::return_type::OK)
      {
        std::cerr << "Read failed!" << std::endl;
        break;
      }

      // Print state every 100ms (10 loops)
      if (loop_count % 10 == 0)
      {
        std::cout << "\033[2J\033[H"; // Clear screen
        std::cout << "=== Kortex3 Read Only Test ===" << std::endl;
        std::cout << "Time: " << std::fixed << std::setprecision(1)
                  << std::chrono::duration<double>(loop_start - start_time).count() << "s" << std::endl;

        // Print Joint States
        std::cout << "\nJoint States:" << std::endl;
        std::cout << "--------------------------------------------------------------" << std::endl;
        std::cout << "Joint | Position [deg] | Velocity [deg/s] | Torque [Nm]" << std::endl;
        std::cout << "------|----------------|------------------|-------------" << std::endl;

        const auto& positions = hw->get_joint_positions();
        const auto& velocities = hw->get_joint_velocities();
        const auto& torques = hw->get_joint_torques();

        if (positions.size() >= 6 && velocities.size() >= 6 && torques.size() >= 6)
        {
            for (size_t i = 0; i < 6; i++)
            {
              // Convert radians to degrees for display
              double pos_deg = positions[i] * 180.0 / M_PI;
              double vel_deg = velocities[i] * 180.0 / M_PI;
              double torque_val = torques[i];

              std::cout << "  " << i + 1 << "   | "
                        << std::setw(14) << std::fixed << std::setprecision(2) << pos_deg << " | "
                        << std::setw(16) << std::fixed << std::setprecision(3) << vel_deg << " | "
                        << std::setw(11) << std::fixed << std::setprecision(3) << torque_val << std::endl;
            }
        }

        // Print Force/Torque Data
        {
            std::lock_guard<std::mutex> lock(g_wrench_mutex); // Lock mutex before accessing global data
            std::cout << "\nEnd-Effector Wrench (tool_frame):" << std::endl;
            std::cout << "--------------------------------------------------------------" << std::endl;
            std::cout << "Force (N)   : X=" << std::setw(8) << std::fixed << std::setprecision(3) << g_latest_wrench.wrench.force.x
                      << " Y=" << std::setw(8) << std::fixed << std::setprecision(3) << g_latest_wrench.wrench.force.y
                      << " Z=" << std::setw(8) << std::fixed << std::setprecision(3) << g_latest_wrench.wrench.force.z << std::endl;
            std::cout << "Torque (Nm) : X=" << std::setw(8) << std::fixed << std::setprecision(3) << g_latest_wrench.wrench.torque.x
                      << " Y=" << std::setw(8) << std::fixed << std::setprecision(3) << g_latest_wrench.wrench.torque.y
                      << " Z=" << std::setw(8) << std::fixed << std::setprecision(3) << g_latest_wrench.wrench.torque.z << std::endl;
        }

        std::cout << "\nPress Ctrl+C to stop..." << std::endl;
      }

      loop_count++;

      // Control loop rate (100Hz)
      std::this_thread::sleep_until(loop_start + std::chrono::milliseconds(10));
    }

    // Deactivation is skipped in this test
    std::cout << "\n=== Test Complete ===" << std::endl;
  }
  catch (const std::exception& e)
  {
    std::cerr << "Exception: " << e.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}