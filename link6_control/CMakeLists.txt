cmake_minimum_required(VERSION 3.8)
project(link6_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros2_control REQUIRED)


install(
        DIRECTORY config
        DESTINATION share/${PROJECT_NAME}
)




ament_package()