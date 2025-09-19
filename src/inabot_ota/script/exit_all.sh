#!/bin/bash

NC='\033[0m'       # Text Reset
BGreen='\033[1;32m' # Green
BYellow='\033[1;33m' # Yellow
BRed='\033[1;31m'    # Red

# Kill Process Function
function kill_process {
    local process_name="$1"
    echo -e "[exit_all.sh] kill_process(): Searching process [${BGreen}$process_name${NC}]..."

    pids=$(pgrep -f "$process_name")

    if [[ -z $pids ]]; then
        echo -e "[exit_all.sh] kill_process(): ${BYellow}Process is not running${NC}"
        return
    fi

    echo -e "[exit_all.sh] kill_process(): Killing PIDs $pids"
    
    # 종료 시도
    if ! kill -9 $pids 2>/dev/null; then
        echo -e "[exit_all.sh] kill_process(): ${BRed}Permission denied. Try running with sudo.${NC}"
        return
    fi

    # 종료 확인
    timeout=3
    elapsed=0
    while :
    do
        if [[ -z $(pgrep -f "$process_name") ]]; then
            echo -e "[exit_all.sh] kill_process(): Successfully terminated $process_name"
            break
        fi
        elapsed=$((elapsed+1))
        if (( elapsed >= timeout )); then
            echo -e "[exit_all.sh] kill_process(): ${BRed}Timeout reached. Process may still be running${NC}"
            break
        fi
        sleep 1
    done
}

###############################################
# Process Names to kill
process_names="\
robot_acs_node \
inabot_core \
robot_state_publisher \
cognex_is8905_node \
static_transform_publisher \
orbbec_camera_node \
component_container \
safety_controller_node \
sick_safetyscanners2_lifecycle_node \
lifecycle_manager \
driver_node \
sick_generic_caller \
sound_node \
stella_ahrs_node \
cartographer_node \
cartographer_occupancy_grid_node \
joy_node \
joy_con \
"

echo -e "[exit_all.sh] Make sure this script has execute permission: chmod +x exit_all.sh"
echo -e "[exit_all.sh] If some processes fail to terminate, try running with sudo"

for process_name in $process_names; do
    kill_process $process_name
done

echo -e "[exit_all.sh] ${BGreen}All processes handled.${NC}"
exit 0
