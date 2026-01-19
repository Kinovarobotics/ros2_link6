# ROS2 Kortex 3
> Kinova® Kortex3™ is the common software platform behind the Link6. It unifies the inner workings of the various robots and their related external tools, like the API.  
> https://www.kinovarobotics.com/en/produit/cobot-link-6

<p align="center">
  <img src="doc/resources/KINOVA_Link6.jpg" alt="Kinova Link6 6DoF manipulator" width="80%"/>
</p>

## 2. Table of Contents

- [ROS2 Kortex 3](#ros2-kortex-3)
  - [2. Table of Contents](#2-table-of-contents)
  - [3. Introduction](#3-introduction)
  - [4. Getting Started](#4-getting-started)
    - [4.1 Prerequisites](#41-prerequisites)
      - [4.1.1 Install Dependencies](#411-install-dependencies)
    - [4.2 Robot Setup](#42-robot-setup)
  - [5. Installation](#5-installation)
    - [5.1 Workspace Layout](#51-workspace-layout)
      - [5.1.1 Create \& enter your workspace](#511-create--enter-your-workspace)
      - [5.1.2 Lay out your `src/` directory](#512-lay-out-your-src-directory)
    - [5.2 Clone Additional Repositories](#52-clone-additional-repositories)
    - [5.3 Dependencies](#53-dependencies)
      - [5.3.1 System Dependencies](#531-system-dependencies)
      - [5.3.2 Package Dependencies](#532-package-dependencies)
    - [5.4 Build \& Source](#54-build--source)
      - [5.4.1 Build](#541-build)
      - [5.4.2 Source](#542-source)
    - [5.5 Extract calibration data](#55-extract-calibration-data)
  - [6. Usage](#6-usage)
    - [6.1 Quick Launch](#61-quick-launch)
      - [6.1.1 Real Hardware](#611-real-hardware)
      - [6.1.2 Simulation](#612-simulation)
    - [6.2 Controllers \& Commands](#62-controllers--commands)
      - [6.2.1 Listing \& Switching Controllers](#621-listing--switching-controllers)
      - [6.2.2 Joint Trajectory Controller](#622-joint-trajectory-controller)
      - [6.2.3 Cartesian Motion Controller](#623-cartesian-motion-controller)
      - [6.2.4 Joint Velocity Controller](#624-joint-velocity-controller)
      - [6.2.5 Robotiq Gripper Controller](#625-robotiq-gripper-controller)
      - [Real-life Control:](#real-life-control)
      - [Simulation Control:](#simulation-control)
    - [6.3 Testing Tools](#63-testing-tools)
      - [6.3.1 Read‑Only Test](#631-readonly-test)
      - [6.3.2 Velocity Control Test](#632-velocity-control-test)
    - [6.4 MoveIt](#64-moveit)
      - [6.4.1 Real Hardware](#641-real-hardware)
      - [6.4.2 Simulation](#642-simulation)
  - [7. Services \& Fault Handling](#7-services--fault-handling)
    - [7.1 Operating Modes](#71-operating-modes)
    - [7.2 Switching Modes](#72-switching-modes)
    - [7.3 Fault Handling](#73-fault-handling)
  - [9. Visualization](#9-visualization)
    - [9.1 RViz Setup](#91-rviz-setup)
    - [9.2 Interactive Marker Control](#92-interactive-marker-control)
    - [9.3 Force/Torque Zeroing](#93-forcetorque-zeroing)
  - [10. Package Overview](#10-package-overview)
    - [link6\_description](#link6_description)
    - [link6\_driver](#link6_driver)
    - [link6\_control](#link6_control)
  - [Future Developments](#future-developments)
  - [Limitations](#limitations)
  - [Authors](#authors)

---

## 3. Introduction
> Kinova® Kortex3™ is the common software platform behind the Link6. It unifies the inner workings of the various robots and their related external tools, like the API.  
> https://www.kinovarobotics.com/en/produit/cobot-link-6

---

## 4. Getting Started

### 4.1 Prerequisites

#### 4.1.1 Install Dependencies

1. **ROS2 Humble** on Ubuntu 22.04  
   Follow the official guide:  
   https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

2. **Ignition Fortress (Gazebo)**  
   See “ROS2 Integration” section here:  
   https://gazebosim.org/docs/fortress/ros_installation/

3. Install wget:
  ```bash
  sudo apt update
  sudo apt install wget -y
  ```
3. **Protobuf v3.20.3**  
   The Kortex API bindings require exactly Protobuf 3.20.3 and to install it from source please run the following commands:
   ```bash
   cd /tmp
   wget https://github.com/protocolbuffers/protobuf/releases/download/v3.20.3/protobuf-cpp-3.20.3.tar.gz
   tar -xzf protobuf-cpp-3.20.3.tar.gz
   cd protobuf-3.20.3
   ./configure --prefix=/usr/local
   make -j$(nproc)
   sudo make install
   sudo ldconfig
   ```
  Finally the installed protobuf version can be verified using the following command:
  ```bash
  /usr/local/bin/protoc --version
  ```

### 4.2 Robot Setup

1. **Power on the Link6** until the status LED is solid green.

2. **Connect Ethernet** to your PC.

3. **Configure your PC’s interface** (Link6 defaults to 192.168.1.10/24):

   ```bash
   sudo ip addr add 192.168.1.20/24 dev <your_eth_if>
   sudo ip link set <your_eth_if> up
   ping 192.168.1.10
   ```

   <p align="center">
     <img src="doc/resources/link6_network_setup.png" alt="Network setup example" width="60%"/>
   </p>

4. **Verify** by browsing to:
   [http://192.168.1.10/dashboard](http://192.168.1.10/dashboard)

   <p align="center">
     <img src="doc/resources/link6_portal.png" alt="Link6 Web Dashboard" width="60%"/>
   </p>

---

## 5. Installation
Install git-lfs to retrieve the kortex 3 library from the pointer:
```bash
sudo apt update
sudo apt install git-lfs
git lfs install
```
### 5.1 Workspace Layout

#### 5.1.1 Create & enter your workspace

```bash
export COLCON_WS=~/workspace/link6_ws
mkdir -p $COLCON_WS/src
cd    $COLCON_WS/src
```

#### 5.1.2 Lay out your `src/` directory

1. Clone this repository:

```bash
git clone https://github.com/Kinovarobotics/ros2_kortex3.git
```
2. Retrieve the kortex 3 library:

```bash
cd ros2_kortex3
git lfs pull
```

3. Rearrange the directories:

```bash
cd $COLCON_WS
mv src/ros2_kortex3/* src/ && rm -rf src/ros2_kortex3
```

At this point, your directories tree should looks as follows:

```
link6_ws/
└── src/
    ├── doc/
    ├── kortex3_hardware/
    ├── link6_bringup/
    ├── link6_control/
    ├── link6_description/
    ├── ros2_kortex3.humble.repos
    ├── LICENSE
    └── README.md
```

### 5.2 Clone Additional Repositories

1. If not already done, make sure that `vcs` is installed:
  ```
  sudo apt install python3-vcstool
  ```
2. Pull the relevant packages:
  ```bash
  cd $COLCON_WS
  vcs import src --skip-existing --input src/ros2_kortex3.$ROS_DISTRO.repos
  ```

2. 

### 5.3 Dependencies

#### 5.3.1 System Dependencies

```bash
# rosdep for resolving dependencies, colcon-clean if you need it later
sudo apt update
sudo apt install \
  python3-rosdep \
  python3-colcon-clean \
  ros-humble-gz-ros2-control \
  ros-humble-gz-ros2-control-demos \
  ros-humble-gripper-controllers

```

#### 5.3.2 Package Dependencies

```bash
cd $COLCON_WS
rosdep update
rosdep install --ignore-src --from-paths src -r -y
```

### 5.4 Build & Source

#### 5.4.1 Build

```bash
colcon build --packages-skip cartesian_controller_simulation cartesian_controller_tests --cmake-args -DCMAKE_BUILD_TYPE=Release
```

#### 5.4.2 Source

Add to your `~/.bashrc` so it’s automatic:

```bash
echo "source $COLCON_WS/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 5.5 Extract calibration data

Each Kinova Link6 is calibrated in the factory. This data can be extracted from the robot and used to apply robot-specific geometric corrections to the URDF files, improving positional accuracy. Although this step is not mandatory, it is highly recommended to avoid end-effector position errors.

We provide a launch file to automatically extract the calibration files and generate the corresponding corrections:

```bash
ros2 launch kortex3_hardware get_calibration.launch.py robot_ip:=192.168.1.10 calibration_dir:=/path/to/calibration_folder
```

When this program is executed it will connect to the robot, download the calibration files, and generate a calibration `.yaml` file into `calibration_dir`. This file can later be used when bringing up the robot (see [6.1 Quick Launch](#61-quick-launch)).

If you have multiple robots, you can use the `output_file` argument to save the files from each robot with a different name. For example:

```bash
ros2 launch kortex3_hardware get_calibration.launch.py robot_ip:=192.168.1.10 calibration_dir:=/path/to/calibration_folder output_file:=robot_1.yaml
```

---

## 6. Usage

### 6.1 Quick Launch

#### 6.1.1 Real Hardware

To bringup a real life Link6 with a mounted robotiq gripper, use the following:

```bash
ros2 launch link6_bringup real_robot.launch.py gripper:=robotiq_2f_85 calibration_file:=/path/to/your_robot_calibration.yaml
```

To bringup the arm without any mounted gripper, use the following:
```bash
ros2 launch link6_bringup real_robot.launch.py calibration_file:=/path/to/your_robot_calibration.yaml
```

**Note:** If no calibration file is specified, the system will use the default calibration from `link6_description/config/default_calibration.yaml`.

#### 6.1.2 Simulation

**Simulation (Ignition/Gazebo Fortress)**

To bringup a simulated Link6 with a mounted robotiq gripper, use the following:

```bash
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$(ros2 pkg prefix link6_description)/share
ros2 launch link6_bringup sim_robot.launch.py gripper:=robotiq_2f_85 calibration_file:=/path/to/calibration/folder/calibration.yaml
```
To bringup the simulated arm without any mounted gripper, use the following:
```bash
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$(ros2 pkg prefix link6_description)/share
ros2 launch link6_bringup sim_robot.launch.py calibration_file:=/path/to/calibration/folder/calibration.yaml
```

**Note:** If no calibration file is specified, the system will use the default calibration from `link6_description/config/default_calibration.yaml`.

<p align="center">
  <img src="doc/resources/link6_gazebo.png" alt="Link6 Gazebo" width="60%"/>
</p>

### 6.2 Controllers & Commands

By default our `controller_manager` brings up:

| Controller                        | Type                                                    | Purpose                          | Input Topic / Interface                                   |
| :-------------------------------- | :------------------------------------------------------ | :------------------------------- | :-------------------------------------------------------- |
| **joint\_state\_broadcaster**     | `joint_state_broadcaster/JointStateBroadcaster`         | Publish all joint states         | `/joint_states` (sensor\_msgs/JointState)                 |
| **joint\_trajectory\_controller**   | `joint_trajectory_controller/JointTrajectoryController`     | Joint trajectory commands | `/joint_trajectory_controller/joint_trajectory` (trajectory_msgs/msg/JointTrajectory) |
| **joint\_velocity\_controller**   | `velocity_controllers/JointGroupVelocityController`     | Low‑level joint‑space velocities | `/joint_velocity_controller/commands` (std_msgs/msg/Float64MultiArray) |
| **cartesian\_motion\_controller** | `cartesian_motion_controller/CartesianMotionController` | Cartesian pose tracking          | `/cartesian_motion_controller/target_frame` (geometry_msgs/msg/PoseStamped)  |
| **robotiq\_gripper\_controller** | `position_controllers/GripperActionController` | Gripper opening/closing control          | `/robotiq_gripper_controller/gripper_cmd` (control_msgs/action/GripperCommand)  |
| **motion\_control\_handle**       | `cartesian_controller_handles/MotionControlHandle`      | RViz interactive‑marker handle   | —                                                         |

#### 6.2.1 Listing & Switching Controllers

```bash
ros2 control list_controllers
```

Switch controllers by activating the desired one and deactivating the not-needed other.

For example: Activate Cartesian motion controller and stop joint velocity controller:

```bash
ros2 control switch_controllers \
  --activate cartesian_motion_controller \
  --deactivate  joint_velocity_controller
```

#### 6.2.2 Joint Trajectory Controller

> **Use‑case:** direct joint trajectory commands.
> **Type:** `joint_trajectory_controller/JointTrajectoryController`

1. Activate (see above).
2. Publish a trajectory:

  ```bash
  ros2 topic pub /joint_trajectory_controller/joint_trajectory trajectory_msgs/JointTrajectory "{
    joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6],
    points: [
      { positions: [0, 0, 0, 0, 0, 0], time_from_start: { sec: 10 } },
    ]
  }" -1
  ```

#### 6.2.3 Cartesian Motion Controller

> **Use‑case:** smooth end‑effector motion via RViz handle or programmatic targets.
> **Type:** `cartesian_motion_controller/CartesianMotionController`

1. **Activate** it (see above).

2. **Send target pose**:

```bash
ros2 topic pub --once /cartesian_motion_controller/target_frame geometry/msgs/msg/PoseStamped "{header: {frame_id: 'base_link'}, pose: {position: {x: 0.5, y: 0.0, z: 0.4}, orientation: {x: -0.766, y: 0.642, z: 0.0, w: 0.0}}}"
```

#### 6.2.4 Joint Velocity Controller

> **Use‑case:** direct joint‑space velocity commands.
> **Type:** `velocity_controllers/JointGroupVelocityController`

1. Activate (see above).
2. Publish velocities:

   ```bash
   ros2 topic pub /joint_velocity_controller/commands std/msgs/msg/Float64MultiArray "{ data: [0, 0, 0, 0, 0, 0.1] }" -r 1
   ```
   
Ensure your `data` array matches the `joints:` ordering in your controller yaml.
**NOTE:** Make sure that whenever you send joint velocities, you send a zero velocity command afterward; otherwise the robot will keep moving based on the last velocity sent.

#### 6.2.5 Robotiq Gripper Controller
> **Use‑case:** control of the opening/closing of a mounted robotiq gripper

> **Type:** `position_controllers/GripperActionController`

#### Real-life Control:

0. Before commanding the gripper, please make sure to use the webapp to install the robotiq_plugin, add the gripper to the list of active tools and activate the gripper using a custom program each time the robot is turned on.

1. Fully open the gripper:
```bash
ros2 action send_goal /robotiq_gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{command:{position: 0.0, max_effort: 100.0}}"
```

2. Fully close the gripper:
```bash
ros2 action send_goal /robotiq_gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{command:{position: 0.81, max_effort: 100.0}}"
```

3. You can partially open the gripper by calling the Action server with the previous command and setting the desired position of the gripper to any number between 0.0 (Fully Open) and 0.81 (Fully Closed), for example:
```bash
ros2 action send_goal /robotiq_gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{command:{position: 0.5, max_effort: 100.0}}"
```

#### Simulation Control:

1. Fully open the gripper:
```bash
ros2 action send_goal /robotiq_gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{command:{position: 0.1, max_effort: 100.0}}"
```

2. Fully close the gripper:
```bash
ros2 action send_goal /robotiq_gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{command:{position: 0.7, max_effort: 100.0}}"
```

3. You can partially open the gripper by calling the Action server with the previous command and setting the desired position of the gripper to any number between 0.0 (Fully Open) and 0.81 (Fully Closed), for example:
```bash
ros2 action send_goal /robotiq_gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{command:{position: 0.5, max_effort: 100.0}}"
```

### 6.3 Testing Tools

#### 6.3.1 Read‑Only Test

**Run the test:**

```bash
ros2 run kortex3_hardware test_read_only
```

**Output:**

```
Press Ctrl+C to stop...
=== Kortex3 Read Only Test ===
Time: 6.4s

Joint States:
--------------------------------------------------------------
Joint | Position [deg] | Velocity [deg/s] | Torque [Nm]
------|----------------|------------------|-------------
  1   |          21.08 |            0.000 |       2.414
  2   |          51.44 |            0.000 |     -16.377
  3   |         115.39 |            0.000 |     -19.640
  4   |           6.16 |            0.000 |      -3.285
  5   |         -16.76 |            0.000 |       4.426
  6   |           2.83 |            0.000 |       1.670

End-Effector Wrench (tool_frame):
--------------------------------------------------------------
Force (N)   : X=   1.877 Y=  -0.572 Z=  -3.509
Torque (Nm) : X=   0.018 Y=   0.028 Z=  -0.025
```

#### 6.3.2 Velocity Control Test

**Run the test:**

```bash
ros2 run kortex3_hardware test_send_velocity
```

Interactive Menu:

```
5. Select velocity test mode:
   0: Zero velocities (stop all joints)
   1: Small sine wave (3 deg/s amplitude, 0.5 Hz)
   2: Constant velocity on joint 1 (5 deg/s)
   3: Custom velocities (you specify)
   4: Constant velocity on last joint (5 deg/s)
   5: Strong sine wave (20 deg/s, 0.5 Hz)

Enter mode (0-5): 
```

**Output (Mode 1):**

```
=== Kortex3 Velocity Control Test ===
Mode: Sine wave (3 deg/s, 0.5 Hz)
Time: 7.4s

Joint States:
----------------------------------------------------------------------------
Joint | Pos [deg] | Vel [deg/s] | Torque [Nm] | Cmd Vel [deg/s]
------|-----------|-------------|-------------|----------------
  1   |     22.34 |      -3.049 |      -6.103 |          -2.915
  2   |     52.69 |      -2.847 |     -24.098 |          -2.915
  3   |    116.64 |      -2.831 |     -24.612 |          -2.915
  4   |      7.41 |      -2.909 |      -6.044 |          -2.915
  5   |    -15.51 |      -3.281 |       2.721 |          -2.915
  6   |      4.07 |      -2.801 |      -0.796 |          -2.915

Press Ctrl+C to stop...
```

---

### 6.4 MoveIt

#### 6.4.1 Real Hardware

To use MoveIt with a real life Link6, first bringup the robot:

```bash
ros2 launch link6_bringup real_robot.launch.py gripper:=robotiq_2f_85
```
Then start MoveIt:
```bash
ros2 launch link6_moveit_config move_group.launch.py use_rviz:=true
```

#### 6.4.2 Simulation

**Simulation (Ignition/Gazebo Fortress)**

To use MoveIt with a simulated Link6, first bringup the robot:

```bash
ros2 launch link6_bringup sim_robot.launch.py gripper:=robotiq_2f_85
```
Then start MoveIt:
```bash
ros2 launch link6_moveit_config move_group.launch.py use_sim_time:=true use_rviz:=true
```

## 7. Services & Fault Handling

### 7.1 Operating Modes

| Value | Mode Name       | Description                                            |
| ----: | :-------------- | :----------------------------------------------------- |
|     0 | UNSPECIFIED     | No particular mode; used as a placeholder              |
|     1 | JOG\_MANUAL     | Joint‑by‑joint manual jogging                          |
|     2 | HAND\_GUIDING   | Gravity‑compensation free‑hand guiding                 |
|     3 | HOLD\_TO\_RUN   | Press‑and‑hold to enable motion                        |
|     4 | AUTO            | Fully autonomous execution (default for ROS 2 Control) |
|     5 | MONITORED\_STOP | Safe stop (holds position, zero torque)                |

### 7.2 Switching Modes

Call the `SetOperatingMode` service to change modes at runtime:

```bash
ros2 service call /kortex3_hardware/set_operating_mode \
  kortex3_hardware/srv/SetOperatingMode "{ operating_mode: <MODE_VALUE> }"
```

*Example: switch to **AUTO** (mode 4)*

```bash
ros2 service call /kortex3_hardware/set_operating_mode \
  kortex3_hardware/srv/SetOperatingMode "{ operating_mode: 4 }"
```

### 7.3 Fault Handling

If the arm enters a fault state (e.g. emergency stop, self-collision), you will see:

```
[ERROR] Robot arm is in FAULT ...
```

To clear faults and return the arm to `OPERATIONAL`:

```bash
ros2 service call /kortex3_hardware/clear_faults \
  kortex3_hardware/srv/ClearFaults "{}"
```

On success you’ll get back:

```yaml
success: true
message: "Faults cleared and arm recovered to OPERATIONAL."
```

Note: Some errors might require the user to login into the web app to manual jog the robot out of it.

---

## 9. Visualization

### 9.1 RViz Setup

In a new terminal, source your workspace and run RViz with the provided configuration file. This will load the robot model and all necessary displays.

```bash
ros2 run rviz2 rviz2 \
-d $(ros2 pkg prefix link6_control)/share/link6_control/config/link6_config.rviz
```

<p align="center">
  <img src="doc/resources/rviz_default.png" alt="Default Rviz with RobotModel Loaded." width="60%"/>
</p>

### 9.2 Interactive Marker Control

The robot launches with the cartesian\_motion\_controller active, but the interactive marker handle is off by default. To control the robot by dragging a marker in RViz, you must activate the motion\_control\_handle.

```bash
ros2 control switch_controllers \
  --activate motion_control_handle
```

If the interactive marker is not on the side menu of rviz, then you can add it by clicking on Add button and in the by topic tab, select /tool\_wrench/Wrench.

<p align="center">
  <img src="doc/resources/add_interactive.png" alt="Rviz Add by topic tab" width="30%"/>
</p>

### 9.3 Force/Torque Zeroing

The wrench visual might have non-calibrated force/torque readings and will show something like this:

<p align="center">
  <img src="doc/resources/rviz_force_problem.png" alt="Rviz showing jumping Force and Torque arrows" width="60%"/>
</p>

You can open the web app go on the side menu and select robot. Once that is done select, Force Torque Sensor, and click the ZERO button.

<p align="center">
  <img src="doc/resources/zero_torque_force_sensor.png" alt="Kinova web app zero sensor page" width="60%"/>
</p>

---

## 10. Package Overview

### link6\_description

This package contains the URDF (Unified Robot Description Format), STL and configuration files for the Kortex-compatible robots.

### link6\_driver

This package implements a ROS node that allows communication between a node and a Kinova Link6 robot.

### link6\_control

This package implements the ROS2 Control configurations that are used by the Kortex3\_hardware package.

## Future Developments

- Add End Effector Velocity Control
- Automated calibration of the Torque/Force sensor
- Joint Position Controller
- Add End Effector Impedance Control
- Add tool management
- Automated calibration of the Torque/Force sensor

## Limitations

- Verify communication delays. 
- Reading over UDP and writing over MQTT (max 500 hz). However, ROS2 Control Cycle is set at the link6_control/config/kortex_control.yaml 

## Authors

- Anas Houssaini — Hardware Interface, initial development and ROS2 integration  
- Abed Al Rahman Al Mrad - Robotiq gripper integration
