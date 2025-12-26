---
title: Integration of the costom robot arm (FRcobot robotic arm) with the 2F-85 end-effector (2/2)
date: 2025-12-25 23:00:00 +0900
categories: [Robotics, Isaac sim]
tags: [docker, ubuntu, isaac-sim, ros2, simulation]
media_subpath: /assets/img/2025-12-25/
image: a_3.png
toc: true
comments: true
pin: true
math: false
mermaid: true
---
<hr>
- [previous post]( /posts/Integration_of_the_(FRcobot_robotic_arm)_with_the_2F_85_end_effector1/ )

<br>

## Step 5: URDF Rendering in Isaac Sim

- First, let's launch Isaac Sim

```bash
cd /isaac-sim
./runapp.sh
```
![picture18](28.png)
_Initial screen_

- File - Import

- choose 

``` /workspace/robot_ws/src/frcobot_ros2/fairino_description/urdf/fairino_with_robotiq.urdf ```

![picture19](29.png)
_Configure it like this_

**then, import**

<br>

and ```create - physics - ground plane```

![picture20](30.png)

<br>
<hr>

## Step 6: Isaac-sim Action Graph

- Let’s start by creating the Action Graph first ```create - graphs - action graph```

<br>

- **ros2 context**

- **on playback tick**

- **isaac read simulation time**

- **ros2 subscribe joint state**

- **ros2 publish joint state**

- **ros2 publish clock**

- **articulation controller**

As shown in the image below, load seven items and connect them with lines

![picture21](31.png)

<br>

Once all the lines are connected, click ```ROS2 Subscribe Joint State```

![picture22](32.png)

Change the topic name to ```isaac_joint_commands```. This is the same name that was configured in ```fairino5_with_robotiq_ros2_control.xacro```

<br>

![picture23](33.png)

Similarly, click ```ROS2 Publish Joint State```, change the topic name to ```isaac_joint_states```, and set the target prim to ```root_joint```

<br>

![picture24](34.png)

Click the ```Articulation Controller``` and change only the target prim to ```root_joint```

<br>
<hr>

## Step 7: Intermediate operation test

- Since we’ve gone through so many steps, I’ll first test whether the robot actually works

<br>

First, we need two Docker container bash sessions. Since one terminal running Isaac Sim is already open, open a new terminal tab and run ```sudo docker exec -it isaac-sim bash```

and then,

```bash
cd /workspace/robot_ws
colcon build
source install/setup.bash
ros2 launch robot_ws demo.launch.py
```

![picture25](35.png)

<br>

Now press the Play button in Isaac Sim, then in RViz move the arm and gripper and click ```Plan and Execute```

![picture26](36.png)
_isaac sim_

![picture27](37.png)
_rviz2_

RViz2 moves as expected, but in Isaac Sim the gripper behaves a bit strangely. We’ll fix this in the next step

<br>
<hr>

## Step 8: Gripper joint configuration

- Since we’ve gone through so many steps, I’ll first test whether the robot actually works

<br>

There’s very little reference material for the Robotiq URDF, so I struggled a bit here

<br>

- I completed the gripper configuration by referring to [Rig Closed-Loop Structures](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/robot_setup/rig_closed_loop_structures.html) 

- The key point is that the gripper should follow a tree structure rather than a closed-loop structure

<br>

For now, I’ll modify ```robotiq_85_left_inner_knuckle_joint``` As shown in the image below, in the Property → Physics panel, set the joint to a revolute joint with limits of −180 to 180, and set the mimic joint’s reference joint path to /World/fairino5_with_robotiq/joints/robotiq_85_left_knuckle_joint

![picture28](38.png)

<br>

Do the same for ```robotiq_85_left_finger_tip_joint```, ```robotiq_85_right_inner_knuckle_joint```, ```robotiq_85_right_knuckle_joint```, and ```robotiq_85_right_finger_tip_joint```: set the limits to −180 to 180 and set the mimic joint to ```/World/fairino5_with_robotiq/joints/robotiq_85_left_knuckle_joint```

> **Warning**: Do not modify ```robotiq_85_left_knuckle_joint```.
{: .prompt-warning }

<br>

**Now it looks like everything is done, but there’s still a problem**

![picture29](39.png)

**The red-highlighted part doesn’t have a joint defined, so when the gripper grabs an object, the joint tends to separate or spread apart**

<br>

So I decided to create the joint manually. As shown in the image below, ```Ctrl-click``` to select ```robotiq_85_left_inner_knuckle_joint``` and ```robotiq_85_left_finger_tip_joint```, then create a revolute joint

![picture00](00.png)

<br>

Then click the revolute joint and align its axis to match the joint as shown in the image below by dragging it(You only need to adjust the X and Y axes)

![picture01](01.png)

<br>

Then set the revolute joint axis to Y and enter the limits as −180 to 180 and rename the joint to RevoluteJoint1

![picture02](02.png)

<br>

# **Do the same on the right side as you did on the left, and rename it to RevoluteJoint2**

<br>

- press the Play button in Isaac Sim once to test it

![picture03](03.png)

- You’ll see an error like the one shown. This happens because the joints form a loop. Isaac Sim expects the joint configuration to follow a tree structure, not a closed loop [Rig Closed-Loop Structures](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/robot_setup/rig_closed_loop_structures.html) 

<br>

Current state

```mermaid
flowchart LR
C[inner_knuckle_joint]
    C --> D[knuckle_joint]
    D --> E[finger_tip_joint]
    E --> F[Revolutejoint]
    F --> C
```
To achieve a tree structure, we will disconnect the ```inner knuckle joint```

<br>

- Disconnect the mimic joint for ```robotiq_85_left/right_inner_knuckle_joint``` and check ```Exclude from Articulation``` and set the revolute joint for that joint with ```no limited```

![picture04](04.png)

<br>

and I plan to solve this by using a gear joint ```create - physics - joint - gear joint``` Then place the gear joint under the prim directly below the World

![picture30](40.png)
You’ll see an error like the one shown. This happens because the joints form a loop. Isaac Sim expects the joint configuration to follow a tree structure, not a closed loop

<br>

(gear joint)

- Now set body0 and body1 to ```/World/fairino5_with_robotiq/robotiq_85_left_knuckle_link``` and ```/World/fairino5_with_robotiq/robotiq_85_right_knuckle_link```, and set hinge0 and hinge1 to ```/World/fairino5_with_robotiq/joints/robotiq_85_left_knuckle_joint``` and ```/World/fairino5_with_robotiq/joints/robotiq_85_right_knuckle_joint```

- check ```Exclude from Articulation```

![picture31](41.png)

<br>
<hr>

## Step 9: Test

- We’re almost done, so let’s test it

**first terminal**
```bash
cd /isaac-sim
./runapp.sh
```

**second terminal**
```bash
cd /workspace/robot_ws
colcon build
source install/setup.bash
ros2 launch robot_ws demo.launch.py
```

<br>

![picture32](42.png)

- It ran without any issues. Next, I’ll test picking up an object

![picture33](43.png)

> **Notice**: If it doesn’t grip well, go into robotiq_85_left_knuckle_joint’s drive, change it to force, and try again

<br>

**The remaining issues, like arm shaking or gripper wobbling, can be solved by adjusting the damping and stiffness—then it’s done!**

<br>
<hr>