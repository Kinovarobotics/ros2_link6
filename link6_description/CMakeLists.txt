cmake_minimum_required(VERSION 3.8)
project(link6_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ament_cmake, the build system for ROS 2
find_package(ament_cmake REQUIRED)
find_package(ros2_control REQUIRED)
# Install the package.xml file
install(
        FILES package.xml
        DESTINATION share/${PROJECT_NAME}
)

# Install the entire 'urdf' directory to the share folder
install(
        DIRECTORY urdf
        DESTINATION share/${PROJECT_NAME}
)

install(
        PROGRAMS
                scripts/calibrated_urdf_generator.py
        DESTINATION share/${PROJECT_NAME}/scripts
)

# Install the test script
install(
        PROGRAMS
                scripts/test_calibration.sh
                scripts/test_dynamic_calibration.sh
        DESTINATION share/${PROJECT_NAME}/scripts
)

# Install test directory (contains example calibration files for testing)
# Note: The actual calibration directory is created at runtime by the hardware interface
install(
        DIRECTORY test
        DESTINATION share/${PROJECT_NAME}
)

# Install the entire 'meshes' directory to the share folder
# This is the critical step that will fix your RViz errors.
install(
        DIRECTORY meshes
        DESTINATION share/${PROJECT_NAME}
)


install(
        DIRECTORY worlds
        DESTINATION share/${PROJECT_NAME}
)


install(
        DIRECTORY assets
        DESTINATION share/${PROJECT_NAME}
)

install(
        DIRECTORY config
        DESTINATION share/${PROJECT_NAME}
)

ament_package()