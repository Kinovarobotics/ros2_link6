/**
 * @file get_calibration_node.cpp
 * @brief ROS 2 node to download calibration from Kinova Link6 robot
 *
 * This node connects to the robot using the C++ Kortex API, downloads
 * the calibration data, and generates a YAML file for dynamic loading.
 */

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <filesystem>
#include <memory>
#include <chrono>
#include <thread>

// ROS 2 includes
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

// Kortex API includes
#include "KDetailedException.h"
#include "RouterClient.h"
#include "RouterMQTT.h"
#include "SessionClientRpc.h"
#include "BaseClientRpc.h"
#include "Session.pb.h"

namespace fs = std::filesystem;
namespace k_api = Kinova::Api;

class CalibrationDownloadNode : public rclcpp::Node
{
public:
    CalibrationDownloadNode()
        : Node("calibration_download_node")
    {
        // Declare parameters with defaults
        this->declare_parameter("robot_ip", "");
        this->declare_parameter("port", 1883);
        this->declare_parameter("username", "admin");
        this->declare_parameter("password", "admin");
        this->declare_parameter("output_file", "calibration.yaml");
        this->declare_parameter("calibration_dir", "");

        // Get parameters
        robot_ip_ = this->get_parameter("robot_ip").as_string();
        mqtt_port_ = this->get_parameter("port").as_int();
        username_ = this->get_parameter("username").as_string();
        password_ = this->get_parameter("password").as_string();
        output_filename_ = this->get_parameter("output_file").as_string();
        calibration_dir_ = this->get_parameter("calibration_dir").as_string();

        // Validate required parameters
        if (robot_ip_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "robot_ip parameter is required!");
            throw std::runtime_error("robot_ip parameter is required");
        }

        if (calibration_dir_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "calibration_dir parameter is required!");
            throw std::runtime_error("calibration_dir parameter is required");
        }

        // Determine calibration directory
        setup_calibration_directory();

        // Determine output path
        output_path_ = fs::path(output_filename_);
        if (!output_path_.is_absolute()) {
            output_path_ = calib_dir_ / output_filename_;
        }

        // Print configuration
        print_configuration();

        // Execute the download
        execute_download();

        RCLCPP_INFO(this->get_logger(), "Calibration download completed successfully");
    }

private:
    void setup_calibration_directory()
    {
        calib_dir_ = fs::path(calibration_dir_);

        // Try to create the directory if it doesn't exist to validate the path
        try {
            fs::create_directories(calib_dir_);
            if (!fs::exists(calib_dir_)) {
                RCLCPP_ERROR(this->get_logger(),
                    "Failed to create calibration directory: %s",
                    fs::absolute(calib_dir_).c_str());
                throw std::runtime_error("Failed to create calibration directory");
            }

            // Convert to absolute path for consistent logging
            calib_dir_ = fs::absolute(calib_dir_);
        } catch (const fs::filesystem_error& e) {
            RCLCPP_ERROR(this->get_logger(),
                "Invalid calibration directory path: %s - %s",
                calibration_dir_.c_str(), e.what());
            throw std::runtime_error("Invalid calibration directory path");
        }
    }

    void print_configuration()
    {
        RCLCPP_INFO(this->get_logger(), "============================================================");
        RCLCPP_INFO(this->get_logger(), "Kinova Link6 Calibration Downloader");
        RCLCPP_INFO(this->get_logger(), "============================================================");
        RCLCPP_INFO(this->get_logger(), "Robot IP:          %s:%d", robot_ip_.c_str(), mqtt_port_);
        RCLCPP_INFO(this->get_logger(), "Calibration dir:   %s", calib_dir_.c_str());
        RCLCPP_INFO(this->get_logger(), "Output YAML:       %s", output_path_.c_str());
        RCLCPP_INFO(this->get_logger(), "============================================================");
    }

    void execute_download()
    {
        download_calibration();
        generate_yaml();

        // Print success message
        RCLCPP_INFO(this->get_logger(), "============================================================");
        RCLCPP_INFO(this->get_logger(), "SUCCESS!");
        RCLCPP_INFO(this->get_logger(), "============================================================");
        RCLCPP_INFO(this->get_logger(), "Calibration YAML file created at:");
        RCLCPP_INFO(this->get_logger(), "  %s", output_path_.c_str());
        RCLCPP_INFO(this->get_logger(), "To use this calibration in your launch file:");
        RCLCPP_INFO(this->get_logger(), "  calibration_file:=\"%s\"", output_path_.c_str());
        RCLCPP_INFO(this->get_logger(), "============================================================");
    }

    void download_calibration()
    {
        RCLCPP_INFO(this->get_logger(), "Connecting to robot at %s:%d...",
            robot_ip_.c_str(), mqtt_port_);

        try {
            // 1. Create MQTT connection
            auto router_mqtt = std::make_shared<k_api::RouterMQTT>(robot_ip_, mqtt_port_);
            router_mqtt->SpinProcess(std::chrono::milliseconds{1});

            // 2. Create session
            auto session_mqtt = std::make_shared<k_api::Session::SessionClient>(router_mqtt.get());
            auto mqtt_session_info = k_api::Session::CreateSessionInfo();
            mqtt_session_info.set_username(username_);
            mqtt_session_info.set_password(password_);
            mqtt_session_info.set_session_inactivity_timeout(20000);
            mqtt_session_info.set_connection_inactivity_timeout(10000);
            session_mqtt->CreateSession(mqtt_session_info);

            // 3. Create base client
            auto base_mqtt = std::make_shared<k_api::Base::BaseClient>(router_mqtt.get());

            RCLCPP_INFO(this->get_logger(), "✓ Connected successfully");

            // 4. Download calibration
            RCLCPP_INFO(this->get_logger(), "Downloading calibration data from robot...");
            auto blob = base_mqtt->ExportArmCalibration();

            // 5. Save to ZIP file
            fs::create_directories(calib_dir_);
            fs::path zip_path = calib_dir_ / "link6.zip";

            std::ofstream out(zip_path, std::ios::binary | std::ios::trunc);
            if (!out) {
                RCLCPP_ERROR(this->get_logger(), "Failed to open file for writing: %s",
                    zip_path.c_str());
                throw std::runtime_error("Failed to open ZIP file for writing");
            }

            for (auto b : blob.data()) {
                out.put(static_cast<char>(b));
            }
            out.close();

            RCLCPP_INFO(this->get_logger(), "✓ Calibration bundle saved: %s",
                zip_path.c_str());

            // 6. Extract calib.xml
            xml_path_ = calib_dir_ / "calib.xml";
            RCLCPP_INFO(this->get_logger(), "Extracting calib.xml...");

            std::string cmd = "unzip -oq " + zip_path.string() + " calib.xml -d " + calib_dir_.string();
            if (std::system(cmd.c_str()) != 0) {
                RCLCPP_ERROR(this->get_logger(), "unzip command failed. Make sure 'unzip' is installed");
                throw std::runtime_error("unzip command failed");
            }

            RCLCPP_INFO(this->get_logger(), "✓ Calibration XML extracted: %s",
                xml_path_.c_str());

            // 7. Close session
            RCLCPP_INFO(this->get_logger(), "Closing connection...");
            session_mqtt->CloseSession();
            router_mqtt->SpinProcess(std::chrono::milliseconds{0});
            RCLCPP_INFO(this->get_logger(), "✓ Disconnected from robot");
        }
        catch (const k_api::KDetailedException& ex) {
            RCLCPP_ERROR(this->get_logger(), "Kortex API error: %s", ex.what());
            throw;
        }
        catch (const std::exception& ex) {
            RCLCPP_ERROR(this->get_logger(), "Error: %s", ex.what());
            throw;
        }
    }

    void generate_yaml()
    {
        RCLCPP_INFO(this->get_logger(), "Generating YAML calibration file...");

        // Find the generator script
        std::string hw_share;
        try {
            hw_share = ament_index_cpp::get_package_share_directory("kortex3_hardware");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Could not find kortex3_hardware package");
            throw std::runtime_error("Could not find kortex3_hardware package");
        }

        fs::path generator_script = fs::path(hw_share) / "scripts" / "calibration_yaml_generator.py";

        if (!fs::exists(generator_script)) {
            RCLCPP_ERROR(this->get_logger(), "Generator script not found: %s",
                generator_script.c_str());
            throw std::runtime_error("Generator script not found");
        }

        // Run the generator script
        std::ostringstream cmd;
        cmd << "python3 " << generator_script
            << " --calibration_file " << xml_path_
            << " --output_file " << output_path_;

        int result = std::system(cmd.str().c_str());

        if (result != 0) {
            RCLCPP_ERROR(this->get_logger(), "YAML generation failed");
            throw std::runtime_error("YAML generation failed");
        }
    }

    // Parameters
    std::string robot_ip_;
    int mqtt_port_;
    std::string username_;
    std::string password_;
    std::string output_filename_;
    std::string calibration_dir_;

    // Paths
    fs::path calib_dir_;
    fs::path output_path_;
    fs::path xml_path_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<CalibrationDownloadNode>();
        rclcpp::shutdown();
        return 0;
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("calibration_download"), "Fatal error: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
}
