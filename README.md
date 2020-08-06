# EMI Internship Project
A ROS framework for comparison of visual-inertial navigation with varying quality gyroscopes in simulated and real turtlebots. 

## Table of Contents
[Introduction](#Introduction)

[Setup](#Setup)

[Running Simulations](#Running-Simulations)

[Running Experiments with Real Turtlebot](#Running-Experiments-with-Real-Turtlebot)

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
If build is complete, this section was successful

To edit the firmware of the robot, first install the arduino IDE, clone the modified code from the github, make your edits, and upload to the robot. 
1. Install the arduino IDE and relevant packages with sections 4.1.1-4.1.6 of [this tutorial](https://emanual.robotis.com/docs/en/parts/controller/opencr10/#arduino-ide). 
1. Clone the [github with the firmware developed for EMI](https://github.com/woodjosh/EMI-Turtlebot-Firmware) into your Arduino workspace.
    ```
    $ cd ~/Arduino/
    $ git clone https://github.com/woodjosh/EMI-Turtlebot-Firmware 
    ```
1. Make whatever changes are desired to the 3 files in the EMI-Turtlebot-Firmware folder. 
1. Upload to the robot by connecting with micro-usb to the OpenCR board in the second layer of the turtlebot, opening the my_turtlebot3_core.ino document in the Arduino IDE and pressing the upload button. If `jump_to_fw` is displayed, the upload was successful. 

Make sure the analog gyroscopes are plugged in to the OpenCR board correctly. They are plugged into the analog pins on the board, highlighted in the image below. 
<img src="https://github.com/woodjosh/EMI-Internship/blob/master/images/portsfromabove.png">

The wires for the current set up are shown from a side view below. They are also marked on the OpenCR PCB. 
<img src="https://github.com/woodjosh/EMI-Internship/blob/master/images/gyroports.png">

## Running Simulations  
Running tests in simulation has 5 steps: 
1. [Drive robot through environment with keyboard control and record velocity commands ](#Drive-robot-through-environment-with-keyboard-control)
1. [Create robot model with desired IMU settings ](#Create-robot-model-with-desired-IMU-settings) 
1. [Drive robot through environment using recorded velocity commands and record images, imu data, and ground truth data](#Drive-robot-through-environment-and-record-data) 
1. [Run robot_localization package on dataset to visualize and record path estimation results](#Run-robot_localization-package-on-dataset)
1. [Visualize and analyze data in MATLAB](#Visualize-and-analyze-simulated-data-in-MATLAB) 

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

### Visualize and analyze simulated data in MATLAB 
The scripts for data analysis in MATLAB are [here](https://github.com/woodjosh/EMI-Internship-MATLAB/tree/master/Simulation%20Processing). They load data from bag files, display the paths, and calculate RMSE. 

## Running Experiments with Real Turtlebot 
Running experiments with the real turtlebot has 6 steps: 
1. [Configure settings of the path estimation node](#Configure-settings-of-the-path-estimation-node)
1. [Connect to and initialize the turtlebot via ssh](#Connect-to-and-initialize-the-turtlebot-via-ssh)
1. [Record video of robot traveling through environment](#Record-video-of-robot-traveling-through-environment)
1. [Launch and record output of the path estimation nodes](#Launch-and-record-output-of-the-path-estimation-nodes)
1. [Drive robot through environment using teleoperation](#Drive-robot-through-environment-using-teleoperation)
1. [Shut everything down](#Shut-everything-down)
1. [Visualize and analyze data in MATLAB](#Visualize-and-analyze-real-data-in-MATLAB)

### Configure settings of the path estimation node 
The path estimation node is a kalman filter, so it requires tuning for proper functioning. The documentation of the robot_localization node used for this path estimation can be found [here](http://docs.ros.org/melodic/api/robot_localization/html/index.html). [Parameter files](/robot_localization/params/) are used to control how each individual node functions. These are pointed to in the [launch file](/my_pkgs/launch/vio_turtlebot.launch#L26). 

The [launch file](/my_pkgs/launch/vio_turtlebot.launch) controls the launching of all related nodes needed for this path estimation. The important parameter than can be edited here is the [covariance of the visual odometry node](/my_pkgs/launch/vio_turtlebot.launch#L15). This value controls how the filter fuses the visual odometry with the imu odometry. The covariance of the imu messages is approximately 0.002, so I set the visual odometry covariance to 0.0005 because it does not have drift and is more trustworthy. 

### Connect to and initialize the turtlebot via ssh 
Open a new terminal (#1) and start the ros core on your computer

    $ roscore

Power on the turtlebot by flipping the switch on the second layer at the front of the robot. Wait about 30 seconds to allow everything to power up. Open an new terminal (#2) and use [ssh](https://www.ssh.com/ssh/) to connect to the raspberry pi on board the turtlebot.

    $ ssh emi@192.168.0.18 

The password is `turtlebot`. If you have trouble, it is likely a network problem. Some simple google searches will usually resolve the problem. Once connected to the raspberry pi, run the bringup protocol from this same terminal (#2). 

    $ roslaunch turtlebot3_bringup turtlebot3_robot.launch
    
If all is well, the robot will display something similar to the code below.

    SUMMARY
    ========

    PARAMETERS
     * /rosdistro: melodic
     * /rosversion: 1.12.13
     * /turtlebot3_core/baud: 115200
     * /turtlebot3_core/port: /dev/ttyACM0
     * /turtlebot3_core/tf_prefix: 
     * /turtlebot3_lds/frame_id: base_scan
     * /turtlebot3_lds/port: /dev/ttyUSB0

    NODES
      /
        turtlebot3_core (rosserial_python/serial_node.py)
        turtlebot3_diagnostics (turtlebot3_bringup/turtlebot3_diagnostics)
        turtlebot3_lds (hls_lfcd_lds_driver/hlds_laser_publisher)
    ... 
    
    ...
    [INFO] [1531306696.415140]: This core(v1.2.1) is compatible with TB3 Burger
    [INFO] [1531306696.418398]: --------------------------
    [INFO] [1531306696.421749]: Start Calibration of Gyro
    [INFO] [1531306698.953226]: Calibration End
    
### Record video of robot traveling through environment 
This video is used to get a ground truth path for the robot with image processing. It relies on a QR code to track the robot. The video must be stable and unmoving and taken from directly above the driving space of the robot. Any webcam can be used. I took the video with my iphone by using [droidcam](https://www.dev47apps.com), which worked very well for me. Below is an example of an image taken during my experiments. The video should be started **before** the next steps. 

<img src="https://github.com/woodjosh/EMI-Internship/blob/master/images/turtlebotcam_img.png" width="635" height="477.5">

### Launch and record output of the path estimation nodes 
Open a new terminal (#3) and launch the path estimation nodes. These will take the sensor messages from the robot, use them to estimate the robot's path and display the estimated path in rviz. 
    
    $ roslaunch my_pkgs vio_turtlebot.launch
    
Open another terminal (#4) and record the outputs of these nodes to a bag file.

    $ rosbag record /localization/imu_enc /localization/orbslam /localization/vio
    
If there are other topics you want to record, just add them to the end of the command. 

### Drive robot through environment using teleoperation 
Open a final terminal (#5) and start teleoperation. 

    $ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch 
    
At this point, you should have 5 different terminals open at the same time, and your screen should look like this: 
<img src="https://github.com/woodjosh/EMI-Internship/blob/master/images/TerminalsScreenshot.png">

### Shut everything down 
In order to end the trial, things should be shut down in this order: 
1. Stop the robot by pressing Ctrl+C in terminal #5 
1. Stop the rosbag recording by pressing Ctrl-C in terminal #4 
1. Stop the video recording with whatever you were using to record the video 
1. Stop the localization nodes by pressing Ctrl-C in terminal #3 
1. Stop the turtlebot by pressing Ctrl-C in terminal #2 
1. Shut down the turtlebot raspberry pi in terminal #2: `$ sudo shutdown now` password is still turtlebot 
1. Turn off the turtlebot by flipping the switch on the front 
1. End the ros core by pressing Ctrl-C in terminal #1

At this point, everything is properly shut down and the data can be analyzed. 

### Visualize and analyze real data in MATLAB 
The MATLAB scripts for analyzing the data can be found [here](https://github.com/woodjosh/EMI-Internship-MATLAB/blob/master/Real%20Experiment%20Processing). First, run the [process real experiments script](https://github.com/woodjosh/EMI-Internship-MATLAB/blob/master/Real%20Experiment%20Processing/process_real_experiments.m). Make sure to update all filenames at the top of the script so they point to the video file and bag file from the experiment(s) you are looking at and a good destination for the processed data. Once that script has completed, you can run the [plot real results script](https://github.com/woodjosh/EMI-Internship-MATLAB/blob/master/Real%20Experiment%20Processing/plot_real_results.m). Again, make sure the file at the top of the script is the one you exported from the previous script. 
