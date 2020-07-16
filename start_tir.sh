#!/bin/bash

##############################################################
##  This script must be executed outside the LXC container  ##
##############################################################

# These settings can be changed
USE_ROS_COMMUNICATION=true
USE_SIMULATION=true  # Only usable with "USE_ROS_COMMUNICATION=true"
DO_MEASUREMENT=false  # Only usable with "USE_SIMULATION=false"
LXC_USER_NAME="ubuntu"
LXC_INSTANCE="<container name>"
ADDITIONAL_APP_PARAMETERS="--move-home-at-exit --move-home-at-error"
TEACH_POSITIONS=true  # If "true" positions can be teached, Otherwise the positions in the file "robot_arm_positions.json" will be executed.
CONFIGURATION_FILE_PATH=""  # Empty for the default file path ("/home/ubuntu/robot_arm_positions.json")
SCHED_RUNTIME=50000000
SCHED_DEADLINE=50000000
SCHED_PERIOD=50000000

# Set the correct app mode
if [ "$TEACH_POSITIONS" = true ] ; then
    APP_MODE_PARAMETER="--teach-positions"
else
    APP_MODE_PARAMETER="--repeat-recorded-positions"
fi

if [ "$CONFIGURATION_FILE_PATH" != "" ] ; then
    ADDITIONAL_APP_PARAMETERS=${ADDITIONAL_APP_PARAMETERS}" --positions-file-path='"${CONFIGURATION_FILE_PATH}"'"
fi

# Define functions
close_lxc () {
    printf "\nClosing LXC container ..."
    lxc stop $LXC_INSTANCE --force
    printf "\t\t\t[SUCCESSFUL]\n"
}

# Set the signal handlers
trap "close_lxc; exit" SIGINT SIGTERM SIGHUP

if [ "$(lxc info $LXC_INSTANCE | grep 'Status: Stopped')" != "" ] ; then
    printf "Starting LXC Container ..."
    lxc start $LXC_INSTANCE
    sleep 3
    printf "\t\t\t[SUCCESSFUL]\n"
else
    echo "LXC Container already running"
fi

USER_ID=$(lxc exec $LXC_INSTANCE -- id -u $LXC_USER_NAME)
ROS_APP_PARAMETER=""

# Start the prerequisites for the TIR app
echo "Starting prerequisites for the TIR app ..."

if [ "$USE_ROS_COMMUNICATION" = true ] ; then
    ROS_APP_PARAMETER="--use-ros"
    lxc exec $LXC_INSTANCE -- sudo -S -u $LXC_USER_NAME -i bash -i -c 'nohup roscore &>/dev/null &'
    sleep 3

    # Start the InterbotiX node
    if [ "$USE_SIMULATION" = true ] ; then
        lxc exec $LXC_INSTANCE -- sudo -S -u $LXC_USER_NAME -i bash -i -c 'nohup roslaunch teach_industrial_robots_gazebo interbotix_moveit.launch robot_name:=wx200 use_gazebo:=true world_name:="$(rospack find teach_industrial_robots_gazebo)/worlds/white_page_with_folded_corner.world" use_default_rviz:=false &>/dev/null &'
        sleep 5
        lxc exec $LXC_INSTANCE -- sudo -S -u $LXC_USER_NAME -i bash -i -c 'rosservice call /gazebo/unpause_physics'
    else
        lxc exec $LXC_INSTANCE -- sudo -S -u $LXC_USER_NAME -i bash -i -c 'nohup roslaunch teach_industrial_robots_gazebo interbotix_moveit.launch robot_name:=wx200 use_actual:=true world_name:="empty_world.world" use_default_rviz:=false &>/dev/null &'
        sleep 5
    fi
else
    # Initialize the parameter server
    lxc exec $LXC_INSTANCE -- sudo -S -u $LXC_USER_NAME -i bash -i -c "nohup roslaunch interbotix_sdk arm_run.launch robot_name:=wx200 use_default_rviz:=false &>/dev/null &"
    INTERBOTIX_ARM_NODE_PID=$(pidof arm_node)
    sleep 2

    # The InterbotiX node isn't required anymore
    kill -SIGKILL $INTERBOTIX_ARM_NODE_PID
fi

printf "\t\t\t\t\t\t[SUCCESSFUL]\n"

# Start the TIR app
echo "Starting the TIR app ..."
bash lxc exec $LXC_INSTANCE -- sudo -S -u $LXC_USER_NAME -i bash -i -c "export ROS_NAMESPACE=wx200; rosrun teach_industrial_robots teach_industrial_robots_node $APP_MODE_PARAMETER $ROS_APP_PARAMETER $ADDITIONAL_APP_PARAMETERS" & LXC_TIR_APP_PID=$1

sleep 3

declare -i TIR_APP_PID=$(pidof teach_industrial_robots_node)

if [ "$TIR_APP_PID" = 0 ] ; then
    printf "\t\t\t\t\t\t[FAILED]\n"
    printf "\n\nError: TIR app could not be started! Please check the connection to the robot and the depth camera.\n"
    close_lxc
    exit
else
    printf "\t\t\t\t\t\t[SUCCESSFUL]\n"
fi

# Set network constraints for measurement
if [ "$DO_MEASUREMENT" = true ] && [ "$USE_SIMULATION" = false ] ; then
    printf "Setting network constraints for measurement ..."
    lxc exec $LXC_INSTANCE -- sudo -S -u $LXC_USER_NAME -i bash -i -c 'sudo tc qdisc add dev lo root handle 1: tbf rate 940Mbit burst 8192 limit 16384; sudo tc qdisc add dev lo parent 1:1 handle 10 netem corrupt 0.5% loss 0.1% delay 0.2ms 0.05ms distribution normal reorder 3% 50%'
    printf "\t[SUCCESSFUL]\n"
fi

if [ "$TEACH_POSITIONS" = true ] ; then
    # Schedule the TIR app for real time usage
    echo "Setting real time scheduler for TIR app ..."
    PROCESS_SCHEDULING=""
    declare -i INTERBOTIX_ARM_NODE_PID=0
    SET_SCHED_DEADLINE="chrt -d --sched-runtime $SCHED_RUNTIME --sched-deadline $SCHED_DEADLINE --sched-period $SCHED_PERIOD -p 0"

    if [ "$USE_ROS_COMMUNICATION" = true ] && [ "$USE_SIMULATION" = false ] ; then
        INTERBOTIX_ARM_NODE_PID=$(pidof arm_node)

        if [ "$INTERBOTIX_ARM_NODE_PID" -gt 0 ] ; then
            PROCESS_SCHEDULING=${PROCESS_SCHEDULING}"$SET_SCHED_DEADLINE $INTERBOTIX_ARM_NODE_PID"
        else
            PROCESS_SCHEDULING=":"
        fi
    else
        PROCESS_SCHEDULING=":"
    fi

    if [ "$TIR_APP_PID" -gt 0 ] ; then
        PROCESS_SCHEDULING=${PROCESS_SCHEDULING}"; $SET_SCHED_DEADLINE $TIR_APP_PID"
    fi

    pkexec bash -c "sleep 2; $PROCESS_SCHEDULING"
    FAILED_TO_SET_SCHEDULER=false

    if [ "$TIR_APP_PID" -gt 0 ] ; then
        if [ $(chrt -p $TIR_APP_PID | grep -c "SCHED_DEADLINE") -eq 0 ] ; then
            FAILED_TO_SET_SCHEDULER=true
        fi
    fi

    if [ "$INTERBOTIX_ARM_NODE_PID" -gt 0 ] ; then
        if [ $(chrt -p $INTERBOTIX_ARM_NODE_PID | grep -c "SCHED_DEADLINE") -eq 0 ] ; then
            FAILED_TO_SET_SCHEDULER=true
        fi
    fi

    if [ "$FAILED_TO_SET_SCHEDULER" = true ] ; then
        printf "\t\t\t\t\t\t[FAILED]\n"
        printf "\n\nError: Failed to set real time scheduler\n"
        close_lxc
        exit
    else
        printf "\t\t\t\t\t\t[SUCCESSFUL]\n"
    fi

    # Remove traps and set the new signal handlers
    trap > /dev/null
    trap "kill -SIGINT $TIR_APP_PID; sleep 3; close_lxc; exit" SIGINT SIGTERM SIGHUP
fi

# Wait for the TIR app to complete
wait $LXC_TIR_APP_PID

if [ "$TEACH_POSITIONS" = false ] ; then
    close_lxc
fi
