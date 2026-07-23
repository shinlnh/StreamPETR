#!/bin/bash

echo "[INFO] Find and kill ros 2 processes of $USER"
# get pid
ros2_pids=$(pgrep -u "$USER" -f ros2)
ros1_pids=$(pgrep -u "$USER" -f "roscore|rosrun|roslaunch|rosbag|rosnode|rostopic|rosservice|rosparam")

# kill ros 2 processes
if [ -z "$ros2_pids" ]; then
    echo "[INFO] Do not have any ros 2 process."
else
    echo "$ros2_pids"

    # kill all processes
    for pid in $ros2_pids; do
        echo "[INFO] Killing PID $pid"
        kill -9 $pid
    done

    echo "[INFO] Kill all ros 2 process successfully."
fi

# kill ros 1 processes
if [ -z "$ros1_pids" ]; then
    echo "[INFO] Do not have any ros 1 process."
else
    echo "$ros1_pids"

    # kill all processes
    for pid in $ros1_pids; do
        echo "[INFO] Killing PID $pid"
        kill -9 $pid
    done

    echo "[INFO] Kill all ros 1 process successfully."
fi


# ps -o pid=,comm= | grep -vE 'bash|ps|grep|awk' | awk '{print $1}' | xargs kill -9
