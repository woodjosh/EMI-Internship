# EMI Internship Project
A ROS framework for comparison of visual-inertial navigation with varying quality gyroscopes in simulated and real turtlebots. 

## Table of Contents
[Introduction](#Introduction)

[Setup](#Setup)

[Use](#Use)

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
    $ mkdir -p ~/catkin_ws/src
    $ cd ~/catkin_ws/
    $ catkin_make
    ```
1. Get source from this github 
    ```
    $ cd ~/catkin_ws/src/ 
    $ git clone https://github.com/woodjosh/EMI-Internship/ .
    $ cd ~/catkin_ws && catkin_make
    add: 'source ~/catkin_ws/devel/setup.bash' to the last line of the .bashrc file with nano or text editor
    $ nano ~/.bashrc OR gedit ~/.bashrc
    ```
1. Make the get_data.sh script executable  
    ```
    $ cd ~/catkin_ws/src/my_pkgs 
    $ chmod 755 ./get_data.sh
    ```    
1. Build catkin workspace 
    ```
    $ cd ~/catkin_ws/
    $ catkin_make
    ```
If build is complete, you are ready to go 

## Use 
Running tests in simulation has 5 steps: 
2. Drive robot through environment with keyboard control and record velocity commands 
2. Create robot model with desired IMU settings 
2. Drive robot through environment using recorded velocity commands and record images, imu data, and ground truth data 
2. Run robot_localization package on dataset to visualize and record path estimation results
2. Visualize and analyze data in MATLAB 

These steps can be run automatically for a desired number of trials using the [get_data.sh](/my_pkgs/get_data.sh) script. To run this script, run: 

    $ rosrun my_pkgs get_data.sh -l house -s 1 -f 5 
    
Where -l is the location label, -s is the starting trial label, and -f is the end trial label. For example, the above command would run 5 trials for each model in the [get_data.sh file](/my_pkgs/get_data.sh#L26) and automatically record the outputs of the kalman filter to bag files in the [/my_pkgs/outputs directory](/my_pkgs/outputs). The script automatically deletes bag files of the actual camera and imu data because of their large size. The results can be processed using the matlab script [here](https://github.com/woodjosh/EMI-Internship-MATLAB). For a better understanding of this custom script, try following through each step below and finding the associated commands in the script.  

### Drive robot through environment with keyboard control
Launch gazebo in desired environment with the default turtlebot model (no imu noise). You should see the environment come up with a robot. This may take a while the first time you load a new environment.   

    $ roslaunch turtlebot3_gazebo turtlebot3_house.launch model:=waffle 
    
In another terminal run the launch file to enable keyboard control and record velocity commands. The data is stored in a bag file in the [/my_pkgs/velocity_commands directory](/my_pkgs/velocity_commands).  

    $ roslaunch my_pkgs record_velcmd.launch location:=house
    
Press CTRL-C twice in the second terminal to end the keyboard control and recording. 

Press CTRL-C in the first terminal to end the gazebo session (this usually takes a while). 

### Create robot model with desired IMU settings 
*Models already exist for 4 different IMUs: [ADXRS652](https://www.analog.com/en/parametricsearch/11176#/sort=s3,desc&p5100=0.17|10.7&p5177=0.01|0.5&p5173=0|0.25), [3DM-GX5-15](https://www.microstrain.com/content/3dm-gx5-15-vru), [EG-120, and EG-1300](https://emcore.com/product-category/fiber-optic-gyro-fog-sensors-navigation-systems/fiber-optic-gyroscopes-fog-components/#products_main_ct)* 

Copy the urdf description files [turtlebot3_waffle.gazebo.xacro](/turtlebot3/turtlebot3_description/urdf/turtlebot3_waffle.gazebo.xacro) and [turtlebot3_waffle.urdf.xacro](/turtlebot3/turtlebot3_description/urdf/turtlebot3_waffle.urdf.xacro) to new files in the same directory with a descriptive key following 'waffle' like: [turtlebot3_waffleADX.urdf.xacro](/turtlebot3/turtlebot3_description/urdf/turtlebot3_waffleADX.urdf.xacro).

Change the [name of the gazebo file in the urdf file](/turtlebot3/turtlebot3_description/urdf/turtlebot3_waffleADX.urdf.xacro#L4) to match the file you just created.

Change the [imu plugin parameters in the gazebo file](/turtlebot3/turtlebot3_description/urdf/turtlebot3_waffle.gazebo.xacro#L105) to desired values. 

*(optional)* For visualization (to make sure I was running experiments with correct parameters) I also changed the [main color of the robot](/turtlebot3/turtlebot3_description/urdf/turtlebot3_waffleADX.gazebo.xacro#L8) for each model I created. 

### Drive robot through environment and record data 
Launch gazebo in desired environment with the desired turtlebot model (model arg matches the end of your `.xacro` file names). 

    $ roslaunch turtlebot3_gazebo turtlebot3_house.launch model:=waffleADX 
    
In another terminal run the launch file to record the data (make sure you update model and location). The data is stored in a bag file in the [/my_pkgs/datasets directory](/my_pkgs/datasets).  

    $ roslaunch  my_pkgs record_datasets.launch model:=waffleADX location:=house
    
The robot should drive through the environment on a path similar to that you recorded. The recording will end when the velocity commands end.

### Run robot_localization package on dataset
Run robot_localization to display and record the ground truth path and estimated paths based on imu data, visual data, and imu+visual data. The data is stored in a bag file in the [/my_pkgs/outputs directory](/my_pkgs/outputs). Make sure the model and location match those used when recording the dataset.  

    $ roslaunch  my_pkgs record_outputs.launch model:=waffleADX location:=house
    
Generally, the fused data should follow the ground truth most closely, the imu data should display drift, and the visual odometry should be relatively accurate but on the wrong scale. If there are not enough visual features the visual odometry will not initialize, and the terminal will display `Map point vector is empty!` repeatedly. 

### Visualize and analyze data in MATLAB 
The scripts for data analysis in MATLAB are [here](https://github.com/woodjosh/EMI-Internship-MATLAB). They load data from bag files, display the paths, and calculate RMSE. They are currently very raw and not well written but they do the trick and could be much better with a bit of effort. 
