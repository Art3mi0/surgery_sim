#!/bin/bash

sleep 3

rosbag record -O /home/temo/experiment_data/current_participant/$1_recording /overlay_left/output_video/compressed /overlay_right/output_video/compressed /phantom/phantom/pose /phantom/phantom/force_feedback /ur5e/toolpose /tf /tf_static /refhap /refplan /reftraj /pedal /robot_plancloud /user_plancloud /overlay_cloud_l /overlay_cloud_r /robot_plan /user_plan /current_mode /phantom/phantom/pose /phantom/phantom/force_feedback /cropped_pointcloud /path_confidence /filtered_path
