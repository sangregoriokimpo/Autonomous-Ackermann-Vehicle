# Autonomous Ackermann Geometry Vehicle

![alt text](lidarackermann.png)

![alt text](pathmarker.png)

[▶️ Watch demo video](testrunfull.mp4)

ROS2 Humble Ackermann vehicle implementation based off of default gazebo plugins. Includes a LIDAR and RGBD sensor for autonomous driving. Utilizes PID tuning and Kalman Filter for control optimization.

Credit to [@Hisham178](https://github.com/Hisham178) for the basic [Ackermann geometry vehicle](https://github.com/Hisham178/ros2_ackermann).

How to install.
1. Git clone and colcon.
2. Make sure to source your build including gazebo --> source /usr/share/gazebo/setup.sh
3. ```ros2 launch ackermann_gazebo bot.launch.py```
4. ```ros2 run autonomous_driver autonomous_driver``` or ```ros2 run autonomous_driver rgbd_driver``` or ```ros2 run autonomous_driver fusion_driver``` or ```ros2 run autonomous_driver fusion_gap_driver```

# To launch multi vehicle mode
```ros2 launch multirobot.launch.py```
or
```ros2 launch multifusion.launch.py```

#Tune lidar
```
ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args \
  -r cloud_in:=/top_3d_lidar_plugin/out \
  -r scan:=/scan \
  -p target_frame:=base_link \
  -p transform_tolerance:=0.05 \
  -p min_height:=-0.20 \
  -p max_height:=0.20 \
  -p angle_min:=-1.5708 \
  -p angle_max:=1.5708 \
  -p angle_increment:=0.0174533 \
  -p range_min:=0.3 \
  -p range_max:=30.0 \
  -p use_inf:=true
```


