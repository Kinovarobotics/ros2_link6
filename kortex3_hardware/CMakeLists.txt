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
find_package(controller_manager_msgs REQUIRED)

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

# Detect system architecture and select appropriate library
if(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
    set(KORTEX_API_LIB "${KORTEX_SDK_DIR}/lib/release/libKortexApiCpp.a")
    message(STATUS "Detected x86_64 architecture (${CMAKE_SYSTEM_PROCESSOR}), using libKortexApiCpp.a")
elseif(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
    set(KORTEX_API_LIB "${KORTEX_SDK_DIR}/lib/release/libKortexApiCpp_ARM.a")
    message(STATUS "Detected ARM architecture (${CMAKE_SYSTEM_PROCESSOR}), using libKortexApiCpp_ARM.a") 
else()
    message(WARNING "Unknown architecture: ${CMAKE_SYSTEM_PROCESSOR}, defaulting to libKortexApiCpp.a")
    set(KORTEX_API_LIB "${KORTEX_SDK_DIR}/lib/release/libKortexApiCpp.a")
endif()

# --- Private gripper implementation (prebuilt) ---
# The proprietary Robotiq/Modbus/slick gripper code is compiled separately (see
# the kortex3_hardware_private repository) and ships only as a prebuilt shared
# library. The public code talks to it exclusively through
# include/kortex3_hardware/gripper_interface.hpp.
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
  set(KORTEX3_PRIVATE_LIB "${CMAKE_CURRENT_SOURCE_DIR}/lib/release_arm/libkortex3_private.so")
else()
  set(KORTEX3_PRIVATE_LIB "${CMAKE_CURRENT_SOURCE_DIR}/lib/release/libkortex3_private.so")
endif()
if(NOT EXISTS "${KORTEX3_PRIVATE_LIB}")
  message(FATAL_ERROR
    "Missing prebuilt private gripper library: ${KORTEX3_PRIVATE_LIB}\n"
    "Build it from the kortex3_hardware_private repository first.")
endif()

# --- Hardware Interface Library ---
add_library(kortex3_hardware_interface SHARED
  src/kortex3_hardware_interface.cpp
)

add_library(kortex3_hardware_interface_low_level SHARED
  src/kortex3_hardware_interface_low_level.cpp
  src/gripper_controller.cpp
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
        # kortex plugin sdk (public) headers
        "${CMAKE_CURRENT_SOURCE_DIR}/include/kortex_plugin_sdk/utils"
        # json schema validator
        "${json_schema_validator_SOURCE_DIR}/src"
)

target_link_libraries(kortex3_hardware_interface PUBLIC
        "${KORTEX_API_LIB}"
        "${KORTEX3_PRIVATE_LIB}"
        nlohmann_json::nlohmann_json
        pthread
)

target_include_directories(kortex3_hardware_interface_low_level PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  "${KORTEX_SDK_DIR}/include"
  "${KORTEX_SDK_DIR}/include/messages"
  "${KORTEX_SDK_DIR}/include/client"
  "${KORTEX_SDK_DIR}/include/client_stubs"
  "${KORTEX_SDK_DIR}/include/client_stubs/SafetyFunctions"
  "${KORTEX_SDK_DIR}/include/common"
  "${KORTEX_SDK_DIR}/include/google"
  "${KORTEX_SDK_DIR}/include/util"
  "${KORTEX_SDK_DIR}/include/participant"
  "${KORTEX_SDK_DIR}/include/server"
  "${KORTEX_SDK_DIR}/include/server_stubs"
)

target_link_libraries(kortex3_hardware_interface_low_level PUBLIC
  "${KORTEX_SDK_DIR}/lib/release/libKortexApiCpp.a"
  "${KORTEX3_PRIVATE_LIB}"
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
        controller_manager_msgs
)

ament_target_dependencies(kortex3_hardware_interface_low_level PUBLIC
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
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

pluginlib_export_plugin_description_file(
  hardware_interface
  kortex3_hardware_interface_low_level.xml
)

# --- Calibration Download Node ---
add_executable(get_calibration_node src/get_calibration_node.cpp)

target_include_directories(get_calibration_node PUBLIC
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
)

target_link_libraries(get_calibration_node PUBLIC
        "${KORTEX_API_LIB}"
        pthread
)

ament_target_dependencies(get_calibration_node PUBLIC
  rclcpp
  ament_index_cpp
)

# --- Test Executables ---
if(BUILD_TESTING)
    add_executable(test_read_only test/test_read_only.cpp)
    target_link_libraries(test_read_only PRIVATE kortex3_hardware_interface)

    add_executable(test_send_velocity test/test_send_velocity.cpp)
    target_link_libraries(test_send_velocity PRIVATE kortex3_hardware_interface)
endif()

# --- Installation Rules ---
# The plugin libraries dlopen against libkortex3_private.so, installed alongside
# them in lib/. Ensure the loader searches the install lib dir via $ORIGIN.
set_target_properties(
        kortex3_hardware_interface
        kortex3_hardware_interface_low_level
        PROPERTIES INSTALL_RPATH "$ORIGIN")

install(FILES "${KORTEX3_PRIVATE_LIB}" DESTINATION lib)

set(install_targets kortex3_hardware_interface get_calibration_node kortex3_hardware_interface_low_level)
if(BUILD_TESTING)
    list(APPEND install_targets
            test_read_only
            test_send_velocity
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

install(
        FILES kortex3_hardware_interface_low_level.xml
        DESTINATION share/${PROJECT_NAME}
)

# --- Install launch files ---
install(
        DIRECTORY launch/
        DESTINATION share/${PROJECT_NAME}/launch
        FILES_MATCHING PATTERN "*.py"
)

# --- Install scripts ---
install(
        PROGRAMS
                scripts/calibration_yaml_generator.py
        DESTINATION share/${PROJECT_NAME}/scripts
)

# --- Export Targets for Downstream Packages ---
ament_export_include_directories(include)
ament_export_libraries(kortex3_hardware_interface)
ament_export_libraries(kortex3_hardware_interface_low_level)
ament_export_dependencies(
        rclcpp
        rclcpp_lifecycle
        hardware_interface
        pluginlib
        geometry_msgs
        tf2_ros
        rosidl_default_runtime
        controller_manager_msgs
)

ament_package()