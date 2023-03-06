rosnode kill /twist_gate
rosnode kill /lidar_republisher
rosnode kill /lidar_republisher1
rosnode kill /lidar_republisher2
rosnode kill /republish1
rosnode kill /republish2
rosnode kill /base_link_to_velodyne
rosnode kill /config_ndt
rosnode kill /detection/lidar_detector/cluster_detect_visualization_center
rosnode kill /detection/object_tracker/ukf_track_visualization_center
rosnode kill /gnss_localizer
rosnode kill /imm_ukf_pda_center
rosnode kill /lidar_euclidean_cluster_detect_center
rosnode kill /lidar_to_camera
rosnode kill /map_to_mobility
rosnode kill /op_behavior_selector
rosnode kill /op_common_params
rosnode kill /op_global_planner
rosnode kill /op_motion_predictor
rosnode kill /op_trajectory_evaluator
rosnode kill /op_trajectory_generator
rosnode kill /points_map_loader
rosnode kill /pose_relay
rosnode kill /predicted_objects_visualizer
rosnode kill /pure_pursuit
rosnode kill /ray_ground_filter_center
rosnode kill /republish
rosnode kill /twist_filter
rosnode kill /vector_map_loader
rosnode kill /vel_relay
rosnode kill /vision_darknet_detect
rosnode kill /voxel_grid_filter
rosnode kill /world_to_map
rosnode kill /yolo3_rects
rosnode kill /vision_darknet_detect
rosnode kill /ndt_matching

sleep 1
rosnode kill /driving_progress_logger

ssh root@192.168.0.8 "python3 /home/root/scripts/terminate_cubetown_autorunner.py"