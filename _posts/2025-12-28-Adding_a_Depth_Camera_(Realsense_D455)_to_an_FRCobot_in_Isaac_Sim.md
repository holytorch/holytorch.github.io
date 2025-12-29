---
title: Adding a Depth Camera (Realsense D455) to an FRCobot in Isaac Sim 4.5
date: 2025-12-28 16:00:00 +0900
categories: [Robotics, Isaac sim]
tags: [docker, ubuntu, isaac-sim, ros2, simulation]
media_subpath: /assets/img/2025-12-29/
image: image.png
toc: true
comments: true
pin: true
math: false
mermaid: true
---
<hr>

- [Integration of the costom robot arm (FRcobot robotic arm) with the 2F-85 end-effector (2/2)]( /posts/Integration_of_the_(FRcobot_robotic_arm)_with_the_2F_85_end_effector2/ )

<br>

## Introdution

- Today, I’ll explain how to add a depth camera to an existing Isaac Sim robotic arm setup.
In the real world, you would typically install a camera mount on the robot arm and then attach the camera to it. However, for agile development, we’ll skip the physical mount and instead apply an offset transform directly to the robot arm so that the camera moves together with it

<br>
<hr>

## Step 1: Adding a Depth Camera TF to a URDF

**fairino5_v6.urdf**

```xml
... previous code
    <dynamics
      damping="0"
      friction="0" />
    <safety_controller
      soft_lower_limit="-3.0543"
      soft_upper_limit="3.0543"
      k_position="15"
      k_velocity="10" />
  </joint>
  
  <link name="tool0" />
  <joint name="wrist3_to_tool0" type="fixed">
    <origin xyz="0 0 0.1" rpy="0 0 0" />
    <parent link="wrist3_link" />
    <child link="tool0" />
  </joint>
  
  <link name="depth_camera" />
  <joint name="tool0_to_depth_camera" type="fixed">
    <origin xyz="0 0.1 0" rpy="0 0 0" />
    <parent link="tool0" />
    <child link="depth_camera" />
  </joint>
  
</robot>
```

<br>

- As shown in the previous code, add the following:

```xml
<link name="depth_camera" />
<joint name="tool0_to_depth_camera" type="fixed">
  <origin xyz="0 0.1 0" rpy="0 0 0" />
  <parent link="tool0" />
  <child link="depth_camera" />
</joint>
```

Here, an offset of 10 cm is applied only along the Y-axis, so the camera is mounted 10 cm above the tool0 frame based on the tool0 TF.

- Then, rebuild the project

```bash
cd /workspace/robot_ws
colcon build
source install/setup.bash
ros2 launch robot_ws demo.launch.py
```

As shown in the image below, you can confirm that the TF is being published at the correct position

![picture1](1.png)

<br>
<hr>

## Step 2: Creating and Positioning a Camera in Isaac Sim

- Now, we’ll go into Isaac Sim and add the depth camera

```create - sensors - camera and depth sensors - intel - d455```

![picture2](2.png)
![picture3](3.png)

<br>

- Next, since it needs to be positioned ```10 cm``` above ```tool0```, go to:

Edit → Preferences

In the Preferences window, uncheck ```Keep prim world transformation when reparenting```

Then, place the RealSense at: ```/World/fairino5_with_robotiq/wrist3_link/tool0/Realsense```

![picture4](4.png)

- Then, adjust the transform to match the values shown in the image below

![picture5](5.png)

<br>

![picture6](6.png)

- After that, re-enable
```Keep prim world transformation when reparenting```

- Then move the RealSense back to:
```/World/Realsense```
(this step is for placing the RealSense at the desired location)

- Finally, adjust the transform to match the values shown in the image above

<br>

![picture7](7.png)

- After that, as shown in the image above, create a fixed joint between ```wrist3_link``` and the ```RSD455```
(joints can only be created between rigid bodies)

Body 0: ```/World/fairino5_with_robotiq/wrist3_link```

Body 1: ```/World/Realsense/RSD455```

- Make sure both paths are set correctly

**Then, press Play and move the robot using MoveIt—you’ll be able to confirm that the camera is properly attached and moving together with the robot**

<br>
<hr>

## Step 3: Publish camera depth and RGB topics

- As the final step, we will publish it via an Action Graph

<br>

- From the top tab ```tools - robotics - ros2 omingraphs - camera```

![picture8](8.png)

<br>

Here, you first need to set the camera prim path
Set it to ```/World/Realsense/RSD455/Camera_Pseudo_Depth```
Then, for now, set the frame ID to ```D455_link```

![picture9](9.png)

<br>

press Enter

![picture10](10.png)

- In this way, the ROS camera ```Action_Graph``` will appear

<br>

! Now, let’s test it

- First, click Play in Isaac Sim, then open another terminal

```bash
sudo docker exec -it isaac-sim bash

rviz2
```

![picture11](11.png)

<br>

- From here, click ```Add``` in the bottom-left corner

![picture12](12.png)

<br>

- Add the RGB image

![picture13](13.png)

You can see that it’s working well in the bottom-left corner

<br>

- The depth image will likely appear black.
It is not well visualized in ```RViz2```, but depth measurement is working correctly
- You can check the depth image using ```ros2 topic echo /depth```

![picture14](14.png)

**It works well!**

<br>
<hr>