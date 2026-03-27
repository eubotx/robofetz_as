#!/bin/bash

# Source the main setup.bash in the current shell (optional)
# source /home/bjorn/robofetz_as/install/setup.bash
# Note: Each tab will source its own setup.bash independently

# Function to open a tab with sourced setup and command
open_tab() {
    local title="$1"
    local command="$2"
    
    gnome-terminal --tab --title="$title" -- bash -c "cd /home/bjorn/robofetz_as && source install/setup.bash && $command; exec bash" &
    sleep 0.5  # Small delay to ensure tabs open in order
}

# Open all tabs
# open_tab "USB Camera" "ros2 launch arena_perception arena_camera_usb.launch.py"

# Option 1: MOG Background Subtraction
open_tab "Opponent Detection (MOG)" "ros2 launch arena_perception opponent_detection_color.launch.py"

# Option 2: Color-based Detection (commented out by default)
# open_tab "Opponent Detection (Color)" "ros2 launch arena_perception opponent_detection_color.launch.py"

open_tab "Robot Bringup" "ros2 launch robot_bringup robofetz_as.launch.py use_sim:=false"
open_tab "Cmd Vel Muxer" "ros2 run combat_strategizer cmd_vel_muxer"
open_tab "Xbox Teleop" "ros2 launch robot_bringup xbox_teleop.launch.py"
open_tab "Radio Bridge" "ros2 run robot_bringup radio_bridge_node"
open_tab "Elf Navigator" "ros2 run robot_navigation elf_navigator"
open_tab "Elf Combat Strategizer" "ros2 run combat_strategizer elf_combat_strategizer"

# Special tab for publishing the autonomy topic (will exit after publishing)
gnome-terminal --tab --title="Enable Autonomy" -- bash -c "source /home/bjorn/robofetz_as/install/setup.bash' --once; exec bash" &

# Wait for all background processes to complete
wait
