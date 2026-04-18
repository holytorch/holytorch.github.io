---
title: Setting Up a Simple Underwater Robot Development Environment using Docker (DVL Project_1)
date: 2026-04-18 04:00:00 +0900
categories: [Ros2, Docker, Gazebo]
tags: [docker, ubuntu, ros2, simulation]
media_subpath: /assets/img/2026-04-18/
image: 1.png
toc: true
comments: true
pin: true
math: false
mermaid: true
---
<hr>
<br>

## Prerequisites

Before starting, ensure you have the following installed on your host machine:

**Host OS**
- Linux Ubuntu amd64

**Docker Engine**

**NVIDIA Container Toolkit (for GPU acceleration in Gazebo)**

**A web browser (for the virtual joystick)**

**rocker**

**dockwater**


<br>
<hr>

## Step 0. Project dave environment setting

- I highly recommend using the Docker installation rather than a local setup
- Dealing with dependency conflicts can get extremely complicated and time-consuming
- You can find the complete setup guide at the link below
- Since I am setting up the development environment based on Project DAVE, you can install most of the prerequisites by visiting the link below. Make sure to specifically check the Docker installation section

### [project dave installation](https://dave-ros2.notion.site/?v=d54cc8422868455888cc629d8e6117a9&p=efbf75623fc743e9b0e55c94c211a1dd&pm=s) 

<br>
<hr>

## Step 1. Host (Laptop) 'PRIME Profiles' Configuration

This is the most common cause of GPU-related issues. If your host Ubuntu is set to an "Energy Saving" mode (using only Intel graphics), Docker will never be able to access the NVIDIA GPU, no matter what flags you use.

**How to Check:**
- Run `nvidia-settings` or execute the following command in your host terminal:

```bash
prime-select query
```

**Solution:**
- If the output is ```on-demand``` or ```intel```, you must force it to nvidia mode:

```bash
sudo prime-select nvidia
```

<br>

**⚠️ Note**: After running this command, you must restart your laptop for the changes to take effect.

<br>
<hr>

## Step 2. Fine-tuning the Environment Inside Docker

If you have configured the environment correctly, you will see yourself entering the Docker container using the command below

<br>

- Let’s start by accessing the container first

```bash
source ~/dave_ws/dave_venv/bin/activate

cd ~/dave_ws/src/dockwater

./run.bash -c dave:latest
```

- **Run the ```run.bash``` script with the ```-c``` flag to enable CUDA (GPU acceleration)**

<br>

```bash
ros2 launch dave_demos dave_robot.launch.py z:=-0.5 namespace:=bluerov2 world_name:=dave_ocean_waves paused:=false
```

<br>

**However, if you run the launch command immediately, ArduSub will not be sourced correctly, and the connection between Gazebo and ArduSub will fail. Therefore, you must resolve these environment issues first before proceeding**

![picture2](2.png)

![picture3](3.png)

<br>

Before launching the simulation, you must manually export the necessary paths to ensure that both the **ArduSub SITL binary** and the **Gazebo plugin** are recognized by the system

Every time you start a new Docker session, execute the following commands in order:

- First, update your environment variables so the system can locate the ArduPilot SITL and the Gazebo bridge plugin
```bash
export PATH=$PATH:/opt/ardusub_ws/ardupilot/build/sitl/bin
export GZ_SIM_SYSTEM_PLUGIN_PATH=$GZ_SIM_SYSTEM_PLUGIN_PATH:/opt/ardusub_ws/ardupilot_gazebo/build
ros2 launch dave_demos dave_robot.launch.py \
  z:=-0.5 \
  namespace:=bluerov2 \
  world_name:=dave_ocean_waves \
  paused:=false
```
**By doing this, you will no longer encounter the errors shown in the previous image**

![picture4](4.png)

<br>
<hr>

## Step 4: Synchronizing the Web Joystick Network with Docker

The web-based virtual joystick communicates with the robot inside Docker via WebSockets. To ensure they can "talk" to each other, you must synchronize the network addresses.

**The Problem:**
If Docker is running on a virtual network bridge, your host's browser at `127.0.0.1` might not reach the WebSocket server inside the container

<br>

**The Solution:**

1.  **Check the Container's IP Address:**
    Inside your Docker terminal, run:
    ```bash
    hostname -I
    ```
    This will return an IP like `172.17.0.2`. Note this address down.

2.  **Configure the Virtual Joystick:**
    * Open `~/dave_ws/src/dave/extras/virtual_joystick.html` on your host machine's browser(via vscode or txt).
    * In the **Address** field, replace `127.0.0.1` with the IP address you just found (e.g., `ws://172.17.0.2:8765`).

3. **Establish Connection:**
Instead of manually typing the address every time, you can modify the default value in the HTML source code. **After updating the script part of the HTML with your Docker IP and refreshing the page, the connection issue should be resolved.** Once the status changes to **"Connected"**, your joystick inputs will be successfully translated into `/joy` messages inside the ROS 2 environment.

![picture5](5.png)

![picture6](6.png)

