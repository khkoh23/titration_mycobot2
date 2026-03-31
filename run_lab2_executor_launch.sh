#!/bin/bash
source /opt/ros/galactic/setup.bash
source ~/microros_ws/install/local_setup.bash
source ~/abct2_ws/install/local_setup.bash
ros2 launch titration_robot_syringe syringe_equivalence_executor.launch.py 