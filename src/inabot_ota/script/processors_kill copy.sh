#!/bin/bash

NC='\033[0m'       # Text Reset
Black='\033[0;30m'        # Black
BGreen='\033[1;32m'       # Green
BYellow='\033[1;33m'      # Yellow

# Kill Process Function
function kill_process {
    echo -e "[exit_all.sh] kill_process(): Search process [${BGreen}"$1"${NC}]..."
    ppid=$(pgrep $1 | ps aux | grep -i $1 | grep -v grep)

    if [[ -z $ppid ]]; then
        echo -e "[exit_all.sh] kill_process(): ${BYellow}Search process is not running${NC}"
    else
        kill -9 $ppid
    fi

    timeout=3
    elapsed=0

    while :
    do
        if [[ -z $(pgrep $1) ]]; then
            break
        fi
	elapsed=$((elapsed+1))
	if ( elapsed >= timeout ); then
	    break
	fi
	sleep 1
    done
    echo -e "[exit_all.sh] kill_process(): end"
}

###############################################

# Process Names to killed
process_names="\
odom_init \
odometry_node \
bms_manager \
pio_interaction \
pointcloud_to_laserscan \
laser_scan_merger \
robot_state_publisher \
sick_tim_5xx_1 \
sick_tim_5xx_2 \
y3space_driver \
motor_leadshine \
robot_localization \
lamp_controller \
robot_sound_node \
ros2_control \
controller_manager \
joint_state_publisher \
joint_state_broadcaster \
rviz2 \
navigation2 \
nav2_controller \
nav2_planner \
nav2_recoveries \
nav2_amcl \
map_server \
tf2_ros \
cartographer_node \
cartographer_occupancy_grid_node \
cartographer_ros \
ros2bag \
ros2_topic \
ros2_service \
ros2_node \
"

# Kill any process using /dev/pcanusb32, /dev/pcanusb33, /dev/pcanusb34...
#echo -e "[exit_all.sh] Killing processes using /dev/pcanusb3X devices..."
#for device in /dev/pcanusb3{2..5}; do
#    if [[ -e $device ]]; then
#        echo -e "[exit_all.sh] Killing processes using $device"
#        sudo fuser -k $device
#    fi
#done

for process_name in $process_names; do
    kill_process $process_name
done

echo -e "[processors_kill.sh] exit all.launch end !!"
exit 0


#sudo fuser -k /dev/pcanusb32
