# ROS-AI-week-3-task-1

smart methods internship - ROS and AI desapline 

# Arduino Robot Arm with ROS

This repository provides the setup and instructions to simulate and visualize an Arduino robot arm using Gazebo and RViz. Additionally, it explains different methods of controlling the robot arm: using `joint_state_publisher` and using MoveIt with kinematics.

## Table of Contents

- [Prerequisites](#prerequisites)
- [Setup](#setup)
- [Running the Simulation](#running-the-simulation)
- [Troubleshooting](#troubleshooting)
- [Control Methods](#control-methods)
  - [Controlling the Robot Arm with `joint_state_publisher`](#controlling-the-robot-arm-with-joint_state_publisher)
  - [Controlling the Robot Arm with MoveIt and Kinematics](#controlling-the-robot-arm-with-moveit-and-kinematics)
- [Building Arduino](#building-arduino)
  - [Windows](#windows)
  - [Mac OS X](#mac-os-x)
  - [Linux](#linux)
  - [Ubuntu on WSL](#ubuntu-on-wsl)
- [License](#license)

## Prerequisites

- ROS (Robot Operating System) Noetic
- Gazebo
- RViz
- MoveIt (for advanced control)

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

## Running the Simulation

To run the simulation and visualize the robot arm in RViz, follow these steps:

1. **Launch the Gazebo simulation**:
    ```sh
    roslaunch arduino_robot_arm robot_arm_gazebo.launch
    ```

2. **Launch RViz with the appropriate configuration**:
    ```sh
    roslaunch arduino_robot_arm robot_arm_rviz.launch
    ```

These commands will start the Gazebo simulation and RViz with the robot state publisher, ensuring that the robot arm is visualized correctly.

## Troubleshooting

If you encounter issues with RViz not displaying the robot correctly, make sure to:

1. **Check if the `robot_state_publisher` node is running**:
    ```sh
    rosnode list
    ```

    You should see `/robot_state_publisher` in the list. If not, start it manually:
    ```sh
    rosrun robot_state_publisher robot_state_publisher
    ```

2. **Ensure the `joint_states` topic is being published**:
    ```sh
    rostopic list
    ```

    You should see `/joint_states` in the list. If not, ensure that your robot model or simulation is publishing joint states.

3. **Use the provided launch files** to avoid manual setup errors:
    ```sh
    roslaunch arduino_robot_arm robot_arm_gazebo.launch
    roslaunch arduino_robot_arm robot_arm_rviz.launch
    ```

## Control Methods

### Controlling the Robot Arm with `joint_state_publisher`

The `joint_state_publisher` node is used to publish the state of the robot's joints. This method is simpler and primarily used for visualization and basic control.

#### How to Use

1. **Start the joint state publisher**:
    ```sh
    rosrun joint_state_publisher joint_state_publisher
    ```

2. **Launch the robot state publisher**:
    ```sh
    rosrun robot_state_publisher robot_state_publisher
    ```

3. **Visualize in RViz**:
    ```sh
    rosrun rviz rviz
    ```

#### Pros and Cons

- **Pros**:
  - Easy to set up.
  - Suitable for simple applications and initial testing.

- **Cons**:
  - Limited control capabilities.
  - Not suitable for complex motion planning.

### Controlling the Robot Arm with MoveIt and Kinematics

MoveIt is a powerful motion planning framework for ROS. It provides advanced tools for controlling robot arms, including inverse kinematics, collision detection, and motion planning.

#### How to Use

1. **Install MoveIt**:
    ```sh
    sudo apt-get install ros-noetic-moveit
    ```

2. **Launch the MoveIt setup for your robot**:
    ```sh
    roslaunch arduino_robot_arm moveit_planning_execution.launch
    ```

3. **Use the MoveIt RViz plugin** to interact with the robot:
    ```sh
    roslaunch arduino_robot_arm moveit_rviz.launch
    ```

4. **Plan and execute motions** using the MoveIt interface in RViz or through Python scripts.

#### Pros and Cons

- **Pros**:
  - Advanced control and motion planning capabilities.
  - Supports collision detection and avoidance.
  - Suitable for complex tasks and robotic applications.

- **Cons**:
  - More complex to set up and configure.
  - Requires understanding of kinematics and motion planning concepts.

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
