# EMI Internship Project
A ROS framework for comparison of visual-inertial navigation with varying quality gyroscopes in simulated and real turtlebots. 

## Table of Contents

## Introduction 
The purpose of this project is to develop a demonstration system to showcase the effectiveness of [Enertia Microsystems Inc.'s](https://enertia-micro.com) novel birdbath resonator gyroscope (BRG) for visual-inertial navigation applications. The BRG is a low-cost, ultra-high precision Micro Electromechanical Systems (MEMS) inertial sensors for future mobility applications (autonomous vehicles).

This project is based on [robot operating system](https://www.ros.org) (ROS), an open source set of software libraries and tools for use in building robot applications. Simulations in [gazebo](http://gazebosim.org) allow testing without a physical robot but the code is translatable to a real robot, and is specifically geared toward a [turtlebot](http://www.robotis.us/turtlebot-3/) outfitted with a monocular camera and an IMU. 

## Setup
To run this project install Ubuntu 18.04 in a virtual machine or dual boot, install ros melodic morenia, and copy files from this github into a catkin_ws
1. Install [Ubuntu 18.04](https://releases.ubuntu.com/18.04/) in a virtual machine or dual boot 

    I did not have sufficient space in a virtual machine to run everything 
1. Install [ROS Melodic Morenia](http://wiki.ros.org/melodic/Installation/Ubuntu) 

    I used 'full desktop install'
1. Install ROS package dependencies 
    ```    
    $ sudo apt-get install ros-melodic-joy ros-melodic-teleop-twist-joy ros-melodic-teleop-twist-keyboard ros-melodic-laser-proc ros-melodic-rgbd-launch ros-melodic-depthimage-to-laserscan ros-melodic-rosserial-arduino ros-melodic-rosserial-python ros-melodic-rosserial-server ros-melodic-rosserial-client ros-melodic-rosserial-msgs ros-melodic-amcl ros-melodic-map-server ros-melodic-move-base ros-melodic-urdf ros-melodic-xacro ros-melodic-compressed-image-transport ros-melodic-rqt-image-view ros-melodic-gmapping ros-melodic-navigation ros-melodic-interactive-markers
    ```

1. Create and build a catkin workspace 
    ```
    $ mkdir -p ~/catkin_ws/src`
    $ cd ~/catkin_ws/`
    $ catkin_make
    ```
1. Get source from this github 
    ```
    $ cd ~/catkin_ws/src/ 
    $ git clone https://github.com/woodjosh/EMI-Internship/ .
    $ cd ~/catkin_ws && catkin_make
    add: source ~/catkin_ws/devel/setup.bash to the last line of the .bashrc file with nano or text editor
    $ nano ~/.bashrc 
    ```
    


## Use 
