#!/bin/bash
function showHelp(){
    echo 
    echo "This script can run END-BEG trials of the same turtlebot simulation in the background (nothing is displayed)"
    echo "Place it in a subfolder of your catkin package"
    echo "and make sure that the file is executable (chmod 755 get_data.sh)"
    echo 
    echo "Run it from command line:"
    echo 
    echo "Use: ./get_data.sh -l [location name] -s [label number of first trial] -f [label number of last trial]"
    echo "Or: rosrun [yourpackage] get_data.sh -l [location name] -s [label number of first trial] -f [label number of last trial]"
    echo "Example: rosrun my_pkgs get_data.sh -l house -s 1 -f 5"
}

# get command line options
while getopts l:n: option
do
case "${option}"
in
l) LOC=${OPTARG};;
s) BEG=${OPTARG};;
f) END=${OPTARG};;
esac
done

models=(waffleADX waffle3DM waffleEG120 waffleEG1300)

for MOD in "${models[@]}"
do
   # record robot data for n trials  
   for (( trial=${BEG}; trial<=${END}; trial++ ))
   do  
      printf "recording %s trial %s data...\n" "${MOD}" "${trial}"
      #bring up world 
      sleep 10 &
      SLEEP_PID=$!
      roslaunch turtlebot3_gazebo turtlebot3_${LOC}.launch model:=${MOD} headless:=true gui:=false &
      #wait for pause
      wait $SLEEP_PID
      #record data 
      roslaunch my_pkgs record_datasets.launch model:="$MOD" location:="$LOC" trial:="$trial" >> ./record_log.txt
      #kill gazebo processes 
      killall -9 gzserver  & killall -9 gzclient
      #record outputs
      printf "recording %s trial %s outputs...\n" "${MOD}" "${trial}"
      roslaunch my_pkgs record_outputs.launch model:="$MOD" location:="$LOC" trial:="$trial" >> ./output_log.txt
      #delete associated data file to save space 
      rm /home/josh/catkin_ws/src/my_pkgs/datasets/${LOC}_${MOD}_${trial}.bag
   done
done 
#play sound 
paplay /usr/share/sounds/freedesktop/stereo/complete.oga

