#!/bin/bash

MAV_TYPE=dragon_ddk
MAV_NAME=ddk
WORLD_FRAME_ID=world
ODOM_TOPIC=ground_truth/odom

echo "MAV name: $MAV_NAME MAV Type: $MAV_TYPE"

MASTER_URI=http://localhost:11311
SETUP_ROS_STRING="export ROS_MASTER_URI=${MASTER_URI}"
SESSION_NAME=sim_exp

CURRENT_DISPLAY=${DISPLAY}
if [ -z ${DISPLAY} ];
then
  echo "DISPLAY is not set"
  CURRENT_DISPLAY=:0
fi

if [ -z ${TMUX} ];
then
  TMUX= tmux new-session -s $SESSION_NAME -d
  echo "Starting new session."
else
  echo "Already in tmux, leave it first."
  exit
fi

# Make mouse useful in copy mode
tmux setw -g mouse on


tmux rename-window -t $SESSION_NAME "Core"
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; roscore" Enter
tmux split-window -t $SESSION_NAME
tmux select-layout -t $SESSION_NAME tiled

tmux new-window -t $SESSION_NAME -n "Main"
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 1; roslaunch exploration_manager tmux_gazebo_sim.launch" Enter
tmux split-window -t $SESSION_NAME
# tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 3; roslaunch exploration_manager tmux_spawn.launch x:=-2.8 y:=-1.5 Y:=1.5708" Enter
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 4; roslaunch exploration_manager tmux_spawn.launch x:=-0.4 y:=1.2 Y:=1.5708" Enter
tmux split-window -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 4; roslaunch exploration_manager tmux_control.launch" Enter
tmux split-window -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 5; export DISPLAY=${CURRENT_DISPLAY}; rosparam set robot_name ${MAV_NAME}; rosrun rqt_mav_manager rqt_mav_manager" Enter
tmux select-layout -t $SESSION_NAME tiled

tmux new-window -t $SESSION_NAME -n "Net"
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 3; roslaunch occ_predictor netnode.launch" Enter
tmux select-layout -t $SESSION_NAME even-horizontal

tmux new-window -t $SESSION_NAME -n "Exp"
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 7; roslaunch exploration_manager tmux_exploration.launch" Enter
tmux select-layout -t $SESSION_NAME even-horizontal

# Add window to easily kill all processes
tmux new-window -t $SESSION_NAME -n "Kill"
tmux send-keys -t $SESSION_NAME "tmux kill-session -t ${SESSION_NAME}"

tmux select-window -t $SESSION_NAME:1
tmux -2 attach-session -t $SESSION_NAME