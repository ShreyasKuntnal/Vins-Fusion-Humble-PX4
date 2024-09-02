#!/usr/bin/env python3

# Import the subprocess and time modules
import subprocess
import time

# List of commands to run
commands = [
    # Run the Micro XRCE-DDS Agent
    "MicroXRCEAgent udp4 -p 8888",

    # Run the PX4 SITL simulation
    # "cd ~/PX4-Autopilot && make px4_sitl gz_x500"

    # Run QGroundControl
    # "cd ~/QGroundControl && ./QGroundControl.AppImage"

    #Run Realsense camera node
    "ros2 launch realsense2_camera rs_launch_custom.py",

    #Run Vins Node
    "ros2 run vins vins_node ~/vins_fusion_ws/src/Vins-Fusion-Humble-PX4/config/realsense_d435i/realsense_stereo_imu_config.yaml"
]

# Loop through each command in the list
for command in commands:
    # Each command is run in a new tab of the gnome-terminal
    subprocess.run(["gnome-terminal", "--tab", "--", "bash", "-c", command + "; exec bash"])
    
    # Pause between each command
    time.sleep(1)