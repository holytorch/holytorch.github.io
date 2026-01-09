---
title: How to Enable ROS 2 Communication Across Separate Docker Containers
date: 2026-01-10 00:00:00 +0900
categories: [Ros2, Docker]
tags: [docker, ubuntu, docker-compose, ros2, simulation]
media_subpath: /assets/img/2026-01-10/
image: 6.png
toc: true
comments: true
pin: true
math: false
mermaid: true
---
<hr>
<br>

## Prerequisites

**Host OS**
- Linux Ubuntu amd64

**Docker**
- Docker Engine

<br>
<hr>

## Introdution

- In real-world robotics projects, it is common to run different components in separate Docker containers.
- To validate and ensure reliable communication between these isolated environments, this post demonstrates how to test ROS 2 inter-container communication using Docker Compose

<br>
<hr>

## Step 1: Create the following directory structure

```bash
cd ~
```

<br>

- Let’s organize the project with the following directory structure, as it is convenient to create it directly in VS Code

```text
docker-compose-test/
├── docker-compose.yml
├── node_a/
│   └── Dockerfile
└── node_b/
    └── Dockerfile
```

<br>
<hr>

## Step 2: Write a Dockerfile and docker-compose.yml

- Create the Dockerfile as shown in the figure below
- Note that osrf/ros:humble-desktop is a preconfigured ROS 2 Humble container image, which allows you to test ROS 2 without installing the environment locally

![picture1](1.png)

<br>

- Additionally, create the ```docker-compose.yml``` file as shown in the figure below

![picture2](2.png)

<br>

- The network name was chosen arbitrarily; containers connected to the same network can communicate with each other

- The ROS 2 Domain ID was also set to the same value

```yml
version: '3.8'

services:
  service-a:
    build: ./node_a
    container_name: talker
    environment:
      - ROS_DOMAIN_ID=0
    networks:
      - my-ros-net
    tty: true
    stdin_open: true

  service-b:
    build: ./node_b
    container_name: listener
    environment:
      - ROS_DOMAIN_ID=0
    networks:
      - my-ros-net
    tty: true
    stdin_open: true

networks:
  my-ros-net:
    driver: bridge  
```

<br>
<hr>

## Step 3: build and test

- Navigate to the project directory and build the containers using the following command:

```bash
cd ~/docker-compose-test
docker compose up --build -d
```

- The ```--build``` option forces Docker to rebuild the images, while the ```-d``` (detached mode) option runs the containers in the background

<br>

![picture3](3.png)

- If the output appears like this, the setup has been completed successfully

<br>

Afterward, verify the running containers using ```docker compose ps```...

![picture4](4.png)

**PERFECT!**

- Open two terminal windows and run the following commands in each one

<br>

first terminal

```bash
docker exec -it talker /bin/bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker
```
![picture5](5.png)

<br>

second terminal

```bash
docker exec -it listener /bin/bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp listener
```
![picture6](6.png)

<br>

- As shown in the figure, communication works correctly

<br>

- If you want to remove the Docker Compose setup, use the following command

```bash
docker compose down
```