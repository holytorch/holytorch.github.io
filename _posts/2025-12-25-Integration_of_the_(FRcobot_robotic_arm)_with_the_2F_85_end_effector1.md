---
title: Integration of the costom robot arm (FRcobot robotic arm) with the 2F-85 end-effector (1/2)
date: 2025-12-25 17:00:00 +0900
categories: [Robotics, Isaac sim]
tags: [docker, ubuntu, isaac-sim, ros2, simulation]
media_subpath: /assets/img/2025-12-25/
image: a_2.png
toc: true
comments: true
pin: true
math: false
mermaid: true
---

## Introdution

- Today, building on our [previous work]( /posts/docker_ubuntu_isaacsim_ros2_setup/ ), I’ll demonstrate how to integrate a custom FRCobot robotic arm with the Robotiq 2F-85 gripper and use MoveIt, complete with rendering in Isaac Sim. I’ll be using the FRCobot URDF file, though there’s very little reference material available for it

<br>
<hr>


## Prerequisites

**Host OS**
- Linux Ubuntu amd64

**Hardware**
- NVIDIA GPU with RTX support
- Minimum 8 GB GPU memory recommended (16 GB+ preferred)
- At least 32 GB system RAM recommended

**GPU Driver**
- NVIDIA driver (version compatible with Isaac Sim 4.5)
- `nvidia-smi` must work correctly on the host

**Docker**
- Docker Engine

**NVIDIA Container Toolkit**
- `nvidia-container-toolkit` installed and configured

<br>
<hr>

## Step 1: Preparation and Workspace Setup

<br>

- Start the Docker container

```bash
xhost +local:
sudo docker start -ai isaac-sim
```
<br>

- Create a folder

```bash
mkdir -p /workspace/robot_ws/src
cd /workspace/robot_ws/src
```

**Refer to the robot arm documentation here:
[fairino docs](https://fairino-doc-en.readthedocs.io/latest/ROSGuide/moveIt2.html)**

<br>

- Clone the repositories

[fairino ros2 repository](https://github.com/FAIR-INNOVATION/frcobot_ros2)

[robotiq repository](https://github.com/PickNikRobotics/ros2_robotiq_gripper)

```bash
git clone -b main https://github.com/FAIR-INNOVATION/frcobot_ros2.git
git clone -b humble https://github.com/PickNikRobotics/ros2_robotiq_gripper.git
```

> **Notice**: Since the Docker container uses Ubuntu 22.04, clone the gripper repository from the Humble branch

**Then it will look like the following image**

![picture1](10.png)

<br>

- Add and Delete

```bash
cd frcobot_ros2
rm -rf fairino20_v6_moveit2_config fairino30_v6_moveit2_config fairino3_v6_moveit2_config fairino3mt_v6_moveit2_config fairino10_v6_moveit2_config fairino16_v6_moveit2_config
cd /workspace/robot_ws/src
cd ros2_robotiq_gripper
git clone -b ros2 https://github.com/tylerjw/serial.git
```
> **Notice**: We will be using fairino5_v6_moveit2_config, so I deleted all the unnecessary ones: fairino20_v6_moveit2_config, fairino30_v6_moveit2_config, fairino3_v6_moveit2_config, fairino3mt_v6_moveit2_config, fairino10_v6_moveit2_config, and fairino16_v6_moveit2_config.
Also, to prevent errors, I cloned the serial repository

<br>

- Test build

```bash
cd /workspace/robot_ws
colcon build
source install/setup.bash
```
**I tested whether the build works, It will probably work fine without any issues**

<br>
<hr>

## Step 2: URDF Modification - Add Tool0

> **Notice**: It’s not super important, but I’m doing it now to make setting the tool center point easier later

<br>

- Purpose: Define an end-effector link capable of mounting a Robotiq gripper

- Details: Add at the very bottom of ```fairino5_v6.urdf```

```bash
cd /workspace/robot_ws/src/frcobot_ros2/fairino_description/urdf
```

open ```fairino5_v6.urdf```

<br>

Add the following code just above ```</robot>```
```xml
  <link name="tool0" />
  <joint name="wrist3_to_tool0" type="fixed">
    <origin xyz="0 0 0.1" rpy="0 0 0" />
    <parent link="wrist3_link" />
    <child link="tool0" />
  </joint>
```
![picture2](11.png)

- If you go into frcobot_description and look at the urdf, you'll see that wrist3_link exists but tool0 (end effector) is not defined.
wrist3_link is where the joint motor is located, so it's not the end effector - you can see this in rviz2.Found the tool0 value: it's 100mm up in the z direction

![picture3](12.webp)

- We will connect them like this: ```tool0 → Robotiq adaptor → Robotiq gripper```

<br>
<hr>

## Step 3: Create FRCobot + Robotiq URDF

- Purpose: Create a XACRO file for connecting the robot and the gripper

- Details: Create fairino_with_robotiq.urdf.xacro

```bash
cd /workspace/robot_ws/src/frcobot_ros2/fairino_description/urdf
touch fairino_with_robotiq.urdf.xacro
```

**Now, add the following code to that file and save it (make sure to understand what you’re adding)**

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://wiki.ros.org/xacro" name="fairino5_with_robotiq">

    <!-- 1. Include the Fairino robot URDF -->
    <xacro:include filename="$(find fairino_description)/urdf/fairino5_v6.urdf" />

    <!-- 2. Include the official Robotiq gripper macro file -->
    <xacro:include filename="$(find fairino_description)/urdf/robotiq_2f_85_macro.urdf.xacro" />

    <!-- 3. Define UR-to-Robotiq adapter macro (first code provided by user) -->
    <xacro:macro name="fr_to_robotiq" params="prefix parent child rotation:=^|${0.0}">
        <joint name="${prefix}fr_to_robotiq_joint" type="fixed">
            <parent link="${parent}"/>
            <child link="${prefix}fr_to_robotiq_link"/>
            <origin xyz="0 0 0" rpy="0 0 ${rotation}"/>
        </joint>

        <link name="${prefix}fr_to_robotiq_link">
            <visual>
                <origin xyz="0 0 0" rpy="0 0 0" />
                <geometry>
                    <mesh filename="package://fairino_description/meshes/visual/2f_85/ur_to_robotiq_adapter.dae"/>
                </geometry>
            </visual>
            <collision>
                <origin xyz="0 0 0" rpy="0 0 0"/>
                <geometry>
                    <mesh filename="package://fairino_description/meshes/collision/2f_85/ur_to_robotiq_adapter.stl"/>
                </geometry>
            </collision>
            <inertial>
                <mass value="0.01" />
                <origin xyz="0 0 0" />
                <inertia ixx="0.000044" ixy="0.0" ixz="0.0" iyy="0.000027" iyz="0.0" izz="0.000027" />
            </inertial>
        </link>

        <joint name="${prefix}gripper_side_joint" type="fixed">
            <parent link="${prefix}fr_to_robotiq_link"/>
            <child link="${child}"/>
            <origin xyz="0 0 0.011" rpy="0 0 0"/>
        </joint>
        <link name="${child}"/>
    </xacro:macro>

    <!-- 4-1. Mount the adapter -->
    <!-- parent: the robot's end (tool0) -->
    <!-- child: virtual link name to connect adapter and gripper (robotiq_coupler) -->
    <xacro:fr_to_robotiq prefix="" parent="tool0" child="robotiq_coupler" rotation="0.0" />

    <!-- 4-2. Mount the gripper -->
    <!-- parent: the end of the adapter created above (robotiq_coupler) -->
    <xacro:robotiq_gripper name="RobotiqGripperHardwareInterface" prefix="" parent="robotiq_coupler" use_fake_hardware="true" com_port="/dev/ttyUSB0">
        <origin xyz="0 0 0" rpy="0 0 0" />
    </xacro:robotiq_gripper>

</robot>

```

_Looking at what we added, the Robotiq files are actually in robotiq_description, but it’s written as fairino_description. We should obviously move the files accordingly_

```bash
cp -r /workspace/robot_ws/src/ros2_robotiq_gripper/robotiq_description/config /workspace/robot_ws/src/frcobot_ros2/fairino_description/
cp -r /workspace/robot_ws/src/ros2_robotiq_gripper/robotiq_description/meshes /workspace/robot_ws/src/frcobot_ros2/fairino_description/
cp /workspace/robot_ws/src/ros2_robotiq_gripper/robotiq_description/urdf/robotiq_2f_85_macro.urdf.xacro /workspace/robot_ws/src/frcobot_ros2/fairino_description/urdf
cd /workspace/robot_ws/src/frcobot_ros2/fairino_description/urdf
```

```bash
cd /workspace/robot_ws/src/frcobot_ros2/fairino_description
```

<br>

**CmakeLists.txt**
```
cmake_minimum_required(VERSION 3.8)
project(fairino_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
#find_package(xacro REQUIRED)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
```

<br>

- Now, you need to go into the ```/workspace/robot_ws/src/frcobot_ros2/fairino_description/robotiq_2f_85_macro.urdf.xacro``` file and fix the file paths

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://wiki.ros.org/xacro" name="robotiq_gripper">
    <xacro:macro name="robotiq_gripper" params="
        name
        prefix
        parent
        *origin
        use_fake_hardware:=false
        mock_sensor_commands:=false
        com_port:=/dev/ttyUSB0
        gripper_speed_multiplier:=1.0
        gripper_force_multiplier:=0.5
        gripper_max_speed:=0.150
        gripper_max_force:=235.0
        gripper_closed_position:=0.7929">

        <link name="${prefix}robotiq_85_base_link">
            <visual>
                <geometry>
                    <mesh filename="package://fairino_description/meshes/visual/2f_85/robotiq_base.dae" />
                </geometry>
            </visual>
            <collision>
                <geometry>
                    <mesh filename="package://fairino_description/meshes/collision/2f_85/robotiq_base.stl" />
                </geometry>
            </collision>
            <inertial>
                <origin xyz="0.0 2.274e-05 0.03232288" rpy="0 0 0" />
                <mass value="6.6320197e-01" />
                <inertia ixx="5.1617816e-04" iyy="5.8802208e-04" izz="3.9462776e-04" ixy="2.936e-8" ixz="0.0" iyz="-3.2296e-7" />
            </inertial>
        </link>

        <link name="${prefix}robotiq_85_left_knuckle_link">
            <visual>
                <geometry>
                    <mesh filename="package://fairino_description/meshes/visual/2f_85/left_knuckle.dae" />
                </geometry>
            </visual>
            <collision>
                <geometry>
                    <mesh filename="package://fairino_description/meshes/collision/2f_85/left_knuckle.stl" />
                </geometry>
            </collision>
            <inertial>
                <origin xyz="0.01213197 0.0002 -0.00058647" rpy="0 0 0" />
                <mass value="1.384773208e-02" />
                <inertia ixx="3.5232e-7" iyy="2.31944e-6" izz="2.23136e-6" ixy="0.0" ixz="1.1744e-7" iyz="0" />
            </inertial>
        </link>

        <link name="${prefix}robotiq_85_right_knuckle_link">
            <visual>
                <geometry>
                    <mesh filename="package://fairino_description/meshes/visual/2f_85/right_knuckle.dae" />
                </geometry>
            </visual>
            <collision>
                <geometry>
                    <mesh filename="package://fairino_description/meshes/collision/2f_85/right_knuckle.stl" />
                </geometry>
            </collision>
            <inertial>
                <origin xyz="-0.01213197 0.0002 -0.00058647" rpy="0 0 0" />
                <mass value="1.384773208e-02" />
                <inertia ixx="3.5232e-7" iyy="2.31944e-6" izz="2.23136e-6" ixy="0.0" ixz="-1.1744e-7" iyz="0.0" />
            </inertial>
        </link>

        <link name="${prefix}robotiq_85_left_finger_link">
            <visual>
                <geometry>
                    <mesh filename="package://fairino_description/meshes/visual/2f_85/left_finger.dae" />
                </geometry>
            </visual>
            <collision>
                <geometry>
                    <mesh filename="package://fairino_description/meshes/collision/2f_85/left_finger.stl" />
                </geometry>
            </collision>
            <inertial>
                <origin xyz="0.00346899 -0.00079447 0.01867121" rpy="0 0 0" />
                <mass value="4.260376752e-02" />
                <inertia ixx="1.385792000000000e-05" iyy="1.183208e-05" izz="5.19672e-06" ixy="0.0" ixz="-2.17264e-06" iyz="0.0" />
            </inertial>
        </link>

        <link name="${prefix}robotiq_85_right_finger_link">
            <visual>
                <geometry>
                    <mesh filename="package://fairino_description/meshes/visual/2f_85/right_finger.dae" />
                </geometry>
            </visual>
            <collision>
                <geometry>
                    <mesh filename="package://fairino_description/meshes/collision/2f_85/right_finger.stl" />
                </geometry>
            </collision>
            <inertial>
                <origin xyz="-0.00346899 -5.53e-06 0.01867121" rpy="0 0 0" />
                <mass value="4.260376752000000e-02" />
                <inertia ixx="1.385792e-05" iyy="1.183208e-05" izz="5.19672e-06" ixy="0.0" ixz="2.17264e-06" iyz="0.0" />
            </inertial>
        </link>

        <link name="${prefix}robotiq_85_left_inner_knuckle_link">
            <visual>
                <geometry>
                    <mesh filename="package://fairino_description/meshes/visual/2f_85/left_inner_knuckle.dae" />
                </geometry>
            </visual>
            <collision>
                <geometry>
                    <mesh filename="package://fairino_description/meshes/collision/2f_85/left_inner_knuckle.stl" />
                </geometry>
            </collision>
            <inertial>
                <origin xyz="0.01897699 0.00015001 0.02247101" rpy="0 0 0" />
                <mass value="2.969376448e-02" />
                <inertia ixx="9.57136e-06" iyy="8.69056e-06" izz="8.19144e-06" ixy="0.0" ixz="-3.93424e-06" iyz="0.0" />
            </inertial>
        </link>

        <link name="${prefix}robotiq_85_right_inner_knuckle_link">
            <visual>
                <geometry>
                    <mesh filename="package://fairino_description/meshes/visual/2f_85/right_inner_knuckle.dae" />
                </geometry>
            </visual>
            <collision>
                <geometry>
                    <mesh filename="package://fairino_description/meshes/collision/2f_85/right_inner_knuckle.stl" />
                </geometry>
            </collision>
            <inertial>
                <origin xyz="-0.01926824 5.001e-05 0.02222178" rpy="0 0 0" />
                <mass value="2.969376448e-02" />
                <inertia ixx="9.42456e-06" iyy="8.69056e-06" izz="8.33824e-06" ixy="0.0" ixz="3.9636e-06" iyz="0.0" />
            </inertial>
        </link>

        <link name="${prefix}robotiq_85_left_finger_tip_link">
            <visual>
                <geometry>
                    <mesh filename="package://fairino_description/meshes/visual/2f_85/left_finger_tip.dae" />
                </geometry>
            </visual>
            <collision>
                <geometry>
                    <mesh filename="package://fairino_description/meshes/collision/2f_85/left_finger_tip.stl" />
                </geometry>
                <surface>
                    <friction>
                    <ode>
                        <mu1>100000.0</mu1>
                        <mu2>100000.0</mu2>
                    </ode>
                    </friction>
                    <contact>
                    <ode>
                        <kp>1e+5</kp>
                        <kd>1</kd>
                        <soft_cfm>0</soft_cfm>
                        <soft_erp>0.2</soft_erp>
                        <minDepth>0.002</minDepth>
                        <maxVel>0</maxVel>
                    </ode>
                    </contact>
                </surface>
            </collision>
            <inertial>
                <origin xyz="-0.01456706 -0.0008 0.01649701" rpy="0 0 0" />
                <mass value="4.268588744e-02" />
                <inertia ixx="1.048152e-05" iyy="1.197888e-05" izz="4.22784e-06" ixy="0.0" ixz="3.5232e-6" iyz="0.0" />
            </inertial>
        </link>

        <link name="${prefix}robotiq_85_right_finger_tip_link">
            <visual>
                <geometry>
                    <mesh filename="package://fairino_description/meshes/visual/2f_85/right_finger_tip.dae" />
                </geometry>
            </visual>
            <collision>
                <geometry>
                    <mesh filename="package://fairino_description/meshes/collision/2f_85/right_finger_tip.stl" />
                </geometry>
                <surface>
                    <friction>
                    <ode>
                        <mu1>100000.0</mu1>
                        <mu2>100000.0</mu2>
                    </ode>
                    </friction>
                    <contact>
                    <ode>
                        <kp>1e+5</kp>
                        <kd>1</kd>
                        <soft_cfm>0</soft_cfm>
                        <soft_erp>0.2</soft_erp>
                        <minDepth>0.002</minDepth>
                        <maxVel>0</maxVel>
                    </ode>
                    </contact>
                </surface>
            </collision>
            <inertial>
                <origin xyz="0.01456706 5e-05 0.01649701" rpy="0 0 0" />
                <mass value="4.268588744e-02" />
                <inertia ixx="1.048152e-05" iyy="1.197888e-05" izz="4.22784e-06" ixy="0.0" ixz="-3.5232e-06" iyz="0.0" />
            </inertial>
        </link>

        <joint name="${prefix}robotiq_85_base_joint" type="fixed">
            <parent link="${parent}" />
            <child link="${prefix}robotiq_85_base_link" />
            <xacro:insert_block name="origin" />
        </joint>

        <joint name="${prefix}robotiq_85_left_knuckle_joint" type="revolute">
            <parent link="${prefix}robotiq_85_base_link" />
            <child link="${prefix}robotiq_85_left_knuckle_link" />
            <axis xyz="0 -1 0" />
            <origin xyz="0.03060114 0.0 0.05490452" rpy="0 0 0" />
            <limit lower="0.0" upper="0.8" velocity="0.5" effort="50" />
        </joint>

        <joint name="${prefix}robotiq_85_right_knuckle_joint" type="revolute">
            <parent link="${prefix}robotiq_85_base_link" />
            <child link="${prefix}robotiq_85_right_knuckle_link" />
            <axis xyz="0 -1 0" />
            <origin xyz="-0.03060114 0.0 0.05490452" rpy="0 0 0" />
            <limit lower="-0.8" upper="0.0" velocity="0.5" effort="50" />
            <mimic joint="${prefix}robotiq_85_left_knuckle_joint" multiplier="-1" />
        </joint>

        <joint name="${prefix}robotiq_85_left_finger_joint" type="fixed">
            <parent link="${prefix}robotiq_85_left_knuckle_link" />
            <child link="${prefix}robotiq_85_left_finger_link" />
            <origin xyz="0.03152616 0.0 -0.00376347" rpy="0 0 0" />
        </joint>

        <joint name="${prefix}robotiq_85_right_finger_joint" type="fixed">
            <parent link="${prefix}robotiq_85_right_knuckle_link" />
            <child link="${prefix}robotiq_85_right_finger_link" />
            <origin xyz="-0.03152616 0.0 -0.00376347" rpy="0 0 0" />
        </joint>

        <joint name="${prefix}robotiq_85_left_inner_knuckle_joint" type="continuous">
            <parent link="${prefix}robotiq_85_base_link" />
            <child link="${prefix}robotiq_85_left_inner_knuckle_link" />
            <axis xyz="0 -1 0" />
            <origin xyz="0.0127 0.0 0.06142" rpy="0 0 0" />
            <mimic joint="${prefix}robotiq_85_left_knuckle_joint" />
        </joint>

        <joint name="${prefix}robotiq_85_right_inner_knuckle_joint" type="continuous">
            <parent link="${prefix}robotiq_85_base_link" />
            <child link="${prefix}robotiq_85_right_inner_knuckle_link" />
            <axis xyz="0 -1 0" />
            <origin xyz="-0.0127 0.0 0.06142" rpy="0 0 0" />
            <mimic joint="${prefix}robotiq_85_left_knuckle_joint" multiplier="-1" />
        </joint>

        <joint name="${prefix}robotiq_85_left_finger_tip_joint" type="continuous">
            <parent link="${prefix}robotiq_85_left_finger_link" />
            <child link="${prefix}robotiq_85_left_finger_tip_link" />
            <axis xyz="0 -1 0" />
            <origin xyz="0.00563134 0.0 0.04718515" rpy="0 0 0" />
            <mimic joint="${prefix}robotiq_85_left_knuckle_joint" multiplier="-1" />
        </joint>

        <joint name="${prefix}robotiq_85_right_finger_tip_joint" type="continuous">
            <parent link="${prefix}robotiq_85_right_finger_link" />
            <child link="${prefix}robotiq_85_right_finger_tip_link" />
            <axis xyz="0 -1 0" />
            <origin xyz="-0.00563134 0.0 0.04718515" rpy="0 0 0" />
            <mimic joint="${prefix}robotiq_85_left_knuckle_joint" />
        </joint>
    </xacro:macro>
</robot>
```
<br>
<hr>

## Step 4: MoveIt2 Setup Assistant Setup
and then,
```bash
cd /workspace/robot_ws
colcon build
source install/setup.bash

ros2 launch moveit_setup_assistant setup_assistant.launch.py
```

<br>

- load ```/workspace/robot_ws/src/frcobot_ros2/fairino_description/urdf/fairino_with_robotiq.urdf.xacro```

![picture3](13.png)

<br>

- Here, just click “Generate Collision Matrix” and move on
![picture4](14.png)

<br>

- then,
![picture5](15.png)

<br>

- then,
![picture6](16.png)
![picture7](17.png)

<br>

- then,
![picture8](18.png)
![picture9](19.png)
![picture10](20.png)

<br>

- then,
![picture11](21.png)
> **Notice**: Setting the gripper open/close poses is mentally beneficial, so let's also set up the robot arm poses

<br>

- end effector
![picture12](22.png)

<br>

- Passive joint
![picture13](23.png)

<br>

- ros2 controller (click "auto add .....")
![picture14](24.png)
> **Notice**: Change the gripper controller type as shown in the picture

<br>

- moveit2 controller (click "auto add .....")
![picture15](25.png)
> **Notice**: Change the gripper controller type as shown in the picture

<br>

- author info
![picture16](26.png)

<br>

- generate files
```bash
mkdir /workspace/robot_ws/src/robot_ws
```
![picture17](27.png)

<br>

- other modifications

```bash
cd /workspace/robot_ws/src/robot_ws/config
```

**joints_limits.yaml**

```yaml
default_velocity_scaling_factor: 0.1
default_acceleration_scaling_factor: 0.1

# Specific joint properties can be changed with the keys [max_position, min_position, max_velocity, max_acceleration]
# Joint limits can be turned off with [has_velocity_limits, has_acceleration_limits]
joint_limits:
  j1:
    has_velocity_limits: true
    max_velocity: 3.1499999999999999
    has_acceleration_limits: true
    max_acceleration: 0.7
  j2:
    has_velocity_limits: true
    max_velocity: 3.1499999999999999
    has_acceleration_limits: true
    max_acceleration: 0.7
  j3:
    has_velocity_limits: true
    max_velocity: 3.1499999999999999
    has_acceleration_limits: true
    max_acceleration: 0.7
  j4:
    has_velocity_limits: true
    max_velocity: 3.2000000000000002
    has_acceleration_limits: true
    max_acceleration: 0.7
  j5:
    has_velocity_limits: true
    max_velocity: 3.2000000000000002
    has_acceleration_limits: true
    max_acceleration: 0.7
  j6:
    has_velocity_limits: true
    max_velocity: 3.2000000000000002
    has_acceleration_limits: true
    max_acceleration: 0.7
  robotiq_85_left_knuckle_joint:
    has_velocity_limits: true
    max_velocity: 0.5
    has_acceleration_limits: true
    max_acceleration: 0.5
```

<br>

- make urdf

```bash
cd /workspace/robot_ws/src/frcobot_ros2/fairino_description/urdf
ros2 run xacro xacro fairino_with_robotiq.urdf.xacro -o fairino_with_robotiq.urdf
```
<br>

**This URDF is being imported into Isaac Sim!**

<br>

For now,

```bash
cd /workspace/robot_ws
colcon build
source install/setup.bash
```

# It looks like this is getting too long, so I’ll write it in the next post

<br>

- [next post]( /posts/Integration_of_the_(FRcobot_robotic_arm)_with_the_2F_85_end_effector2/ )

<br>
