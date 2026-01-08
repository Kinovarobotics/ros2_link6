cmake_minimum_required(VERSION 3.8)
project(kortex3_hardware C CXX)

# --- C++ Standard ---
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- ROS 2 / ament Dependencies ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ament_index_cpp REQUIRED)

include(FetchContent)

FetchContent_Declare(
  json
  URL https://github.com/nlohmann/json/archive/refs/tags/v3.11.2.tar.gz
)

FetchContent_MakeAvailable(json)

# json-schema-validator (depends on nlohmann/json)
FetchContent_Declare(
  json_schema_validator
  GIT_REPOSITORY https://github.com/pboettch/json-schema-validator.git
  GIT_TAG 2.3.0 # or latest
)
FetchContent_MakeAvailable(json_schema_validator)


# --- Service and Message Generation ---
rosidl_generate_interfaces(${PROJECT_NAME}
        "srv/SetOperatingMode.srv"
        "srv/ClearFaults.srv"
        "srv/SimulateEstop.srv"
        "srv/RunProgram.srv"
        "srv/ListPrograms.srv"
        "srv/StopProgram.srv"
        "srv/GetProgramStatus.srv"
        "srv/ListProtectionZones.srv"
        "srv/SetProtectionZoneState.srv"
        "msg/ProgramInfo.msg"
        "msg/ProtectionZoneInfo.msg"
)

# --- Kortex SDK Configuration ---
set(KORTEX_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/kortex3")
add_compile_definitions(_OS_UNIX)

# --- Hardware Interface Library ---
add_library(kortex3_hardware_interface SHARED
        src/kortex3_hardware_interface.cpp
        src/Grippers/FingerGripper.cpp
        src/robotiq_plugin/GripperStatus.cpp
        src/Grippers/RobotiqGripper.cpp
        src/Grippers/VacuumGripper.cpp
        src/slick/Communication/ModbusClientWrapper.cpp
        src/slick/Communication/DeviceCommunication.cpp
        src/slick/Synchronization/WaitableScopeFlag.cpp
        src/slick/Synchronization/ScopeFlag.cpp
        src/slick/Communication/ModbusRequestBuffer.cpp
        
)

target_include_directories(kortex3_hardware_interface PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        # Add Kortex SDK includes
        "${KORTEX_SDK_DIR}/include"
        "${KORTEX_SDK_DIR}/include/client"
        "${KORTEX_SDK_DIR}/include/client_stubs"
        "${KORTEX_SDK_DIR}/include/common"
        "${KORTEX_SDK_DIR}/include/google"
        "${KORTEX_SDK_DIR}/include/messages"
        "${KORTEX_SDK_DIR}/include/participant"
        "${KORTEX_SDK_DIR}/include/server"
        "${KORTEX_SDK_DIR}/include/server_stubs"
        "${KORTEX_SDK_DIR}/include/util"
        # robotiq gripper plugin headers
        "${CMAKE_CURRENT_SOURCE_DIR}/include/robotiq_gripper/Grippers"
        "${CMAKE_CURRENT_SOURCE_DIR}/include/robotiq_gripper/robotiq_plugin"
        # slick headers (root and submodules)
        "${CMAKE_CURRENT_SOURCE_DIR}/include/robotiq_gripper/slick"
        "${CMAKE_CURRENT_SOURCE_DIR}/include/robotiq_gripper/slick/Communication"
        "${CMAKE_CURRENT_SOURCE_DIR}/include/robotiq_gripper/slick/Config"
        "${CMAKE_CURRENT_SOURCE_DIR}/include/robotiq_gripper/slick/File"
        "${CMAKE_CURRENT_SOURCE_DIR}/include/robotiq_gripper/slick/HallOfFame"
        "${CMAKE_CURRENT_SOURCE_DIR}/include/robotiq_gripper/slick/IO"
        "${CMAKE_CURRENT_SOURCE_DIR}/include/robotiq_gripper/slick/Kin"
        "${CMAKE_CURRENT_SOURCE_DIR}/include/robotiq_gripper/slick/Math"
        "${CMAKE_CURRENT_SOURCE_DIR}/include/robotiq_gripper/slick/Python"
        "${CMAKE_CURRENT_SOURCE_DIR}/include/robotiq_gripper/slick/Robot"
        "${CMAKE_CURRENT_SOURCE_DIR}/include/robotiq_gripper/slick/Storage"
        "${CMAKE_CURRENT_SOURCE_DIR}/include/robotiq_gripper/slick/Synchronization"
        "${CMAKE_CURRENT_SOURCE_DIR}/include/robotiq_gripper/slick/TestHelpers"
        "${CMAKE_CURRENT_SOURCE_DIR}/include/robotiq_gripper/slick/Trajectory"
        "${CMAKE_CURRENT_SOURCE_DIR}/include/robotiq_gripper/slick/Utils"
        "${CMAKE_CURRENT_SOURCE_DIR}/include/robotiq_gripper/slick/Uuid"
        # kortex plugin sdk headers
        "${CMAKE_CURRENT_SOURCE_DIR}/include/kortex_plugin_sdk/tool"
        "${CMAKE_CURRENT_SOURCE_DIR}/include/kortex_plugin_sdk/base"
        "${CMAKE_CURRENT_SOURCE_DIR}/include/kortex_plugin_sdk/base/action"
        "${CMAKE_CURRENT_SOURCE_DIR}/include/kortex_plugin_sdk/base/configuration"
        "${CMAKE_CURRENT_SOURCE_DIR}/include/kortex_plugin_sdk/base/state"
        "${CMAKE_CURRENT_SOURCE_DIR}/include/kortex_plugin_sdk/utils"
        # json schema validator
        "${json_schema_validator_SOURCE_DIR}/src"

        
)

target_link_libraries(kortex3_hardware_interface PUBLIC
        "${KORTEX_SDK_DIR}/lib/release/libKortexApiCpp.a"
        nlohmann_json::nlohmann_json
        pthread
)

ament_target_dependencies(kortex3_hardware_interface PUBLIC
        rclcpp
        rclcpp_lifecycle
        hardware_interface
        pluginlib
        geometry_msgs
        tf2_ros
        ament_index_cpp
)

# --- Modern ROS2 Service Linking ---
rosidl_get_typesupport_target(cpp_typesupport_target
        ${PROJECT_NAME} "rosidl_typesupport_cpp"
)
target_link_libraries(kortex3_hardware_interface PUBLIC
        ${cpp_typesupport_target}
)

# --- Plugin Export ---
pluginlib_export_plugin_description_file(
        hardware_interface
        kortex3_hardware_interface.xml
)

# --- Test Executables ---
if(BUILD_TESTING)
    add_executable(test_read_only test/test_read_only.cpp)
    target_link_libraries(test_read_only PRIVATE kortex3_hardware_interface)

    add_executable(test_send_velocity test/test_send_velocity.cpp)
    target_link_libraries(test_send_velocity PRIVATE kortex3_hardware_interface)

    add_executable(test_read_calibration test/test_read_calibration.cpp)
    target_link_libraries(test_read_calibration PRIVATE kortex3_hardware_interface)
endif()

# --- Installation Rules ---
set(install_targets kortex3_hardware_interface)
if(BUILD_TESTING)
    list(APPEND install_targets
            test_read_only
            test_send_velocity
            test_read_calibration
    )
endif()

install(TARGETS ${install_targets}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
        DIRECTORY include/
        DESTINATION include
)

install(
        FILES kortex3_hardware_interface.xml
        DESTINATION share/${PROJECT_NAME}
)

# --- Export Targets for Downstream Packages ---
ament_export_include_directories(include)
ament_export_libraries(kortex3_hardware_interface)
ament_export_dependencies(
        rclcpp
        rclcpp_lifecycle
        hardware_interface
        pluginlib
        geometry_msgs
        tf2_ros
        rosidl_default_runtime
)

ament_package()