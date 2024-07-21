# ROS-AI-week-3-task-1

smart methods internship - ROS and AI discipline 

# Controlling Arduino Robot Arm with ROS (joint state publisher and MoveIt)

This repository provides the setup and instructions to simulate and visualize an Arduino robot arm using Gazebo and RViz. Additionally, it explains different methods of controlling the robot arm: using `joint_state_publisher` and using MoveIt with kinematics.

## Table of Contents

- [Prerequisites](#prerequisites)
- [Setup](#setup)
- [Building Arduino](#building-arduino)
   - [Ubuntu on WSL](#ubuntu-on-wsl)
- [Control Methods](#control-methods)
  - [Controlling the Robot Arm with `joint_state_publisher`](#controlling-the-robot-arm-with-joint_state_publisher)
  - [Controlling the Robot Arm with MoveIt and Kinematics](#controlling-the-robot-arm-with-moveit-and-kinematics)


## Prerequisites

- ROS (Robot Operating System) Noetic
- Gazebo
- RViz
- MoveIt

## Setup

1. **Clone the repository** into your ROS workspace:
    ```sh
    cd ~/catkin_ws/src
    git clone https://github.com/yourusername/arduino_robot_arm.git
    ```

2. **Build the workspace**:
    ```sh
    cd ~/catkin_ws
    catkin_make
    ```

3. **Source the workspace**:
    ```sh
    source devel/setup.bash
    ```
    
## Building Arduino

To build Arduino from source, follow the instructions for your operating system:

### Ubuntu on WSL

1. **Install Development Tools**:
    - Open your WSL terminal and install the necessary tools:
        ```sh
        sudo apt-get update
        sudo apt-get install git make gcc ant openjdk-8-jdk unzip
        ```

2. **Clone the Arduino repository**:
    ```sh
    git clone --depth 1 https://github.com/arduino/Arduino.git
    ```

3. **Build Arduino**:
    ```sh
    cd /path/to/arduino/build
    ant dist
    ```

## Control Methods

### Controlling the Robot Arm with `joint_state_publisher`

The `joint_state_publisher` node is used to publish the state of the robot's joints. This method is simpler and primarily used for visualization and basic control.

    ```sh
    roslaunch robot_arm_pkg check_motors.launch
    roslaunch robot_arm_pkg check_motors_gazebo.launch
    rosrun robot_arm_pkg joint_states_to_gazebo.py
    ```

#### Pros and Cons

- **Pros**:
  - Easy to set up.
  - Suitable for simple applications and initial testing.

- **Cons**:
  - Limited control capabilities.
  - Not suitable for complex motion planning.
 
    <img width="959" alt="ros11" src="https://github.com/user-attachments/assets/4d340bd7-074f-47bc-936b-81b61903aaf8">
   <img width="956" alt="ros22" src="https://github.com/user-attachments/assets/980ca5d3-4df9-46d1-a156-6833fc9d1d4b">
   <img width="959" alt="ros33" src="https://github.com/user-attachments/assets/e031c79f-5565-4035-b648-246734cd8091">

### Controlling the Robot Arm with MoveIt and Kinematics

MoveIt is a powerful motion planning framework for ROS. It provides advanced tools for controlling robot arms, including inverse kinematics, collision detection, and motion planning.

#### How to Use

1. **Install MoveIt**:
    ```sh
    sudo apt-get install ros-noetic-moveit
    ```

2. **Launch the MoveIt setup for your robot**:
    ```sh
    roslaunch moveit_pkg demo_gazebo.launch
    ```

3. **Plan and execute motions** using the MoveIt interface in RViz.

#### Pros and Cons

- **Pros**:
  - Advanced control and motion planning capabilities.
  - Supports collision detection and avoidance.
  - Suitable for complex tasks and robotic applications.

- **Cons**:
  - More complex to set up and configure.
  - Requires understanding of kinematics and motion planning concepts.
 
    <img width="959" alt="ros44" src="https://github.com/user-attachments/assets/079be084-fc3c-411d-978a-b751e13c421e">
    <img width="959" alt="ros55" src="https://github.com/user-attachments/assets/20e09724-c5ae-4c22-86bb-9b7fbfe86f87">
    <img width="959" alt="ros77" src="https://github.com/user-attachments/assets/aefb01a5-d73f-4863-8ec8-67ff32dc05b4">



