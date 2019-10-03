# mpdrl-ros

[![Build Status](https://travis-ci.org/Jumpei-Arima/mpdrl.svg?branch=master)](https://travis-ci.org/Jumpei-Arima/mpdrl)

## prerequisite
You need to convert your PyTorch Model to Torch Script by tracing.
[watch here](https://pytorch.org/tutorials/advanced/cpp_export.html)

If you want to use pointcloud as input, you have to install [pointcloud_to_laserscan](http://wiki.ros.org/pointcloud_to_laserscan) package.
```
$ sudo apt-get install ros-{distro}-pointcloud-to-laserscan
```

# How to use
- /local_map (nav_msgs::OccupancyGrid)
```
$ roslaunch mpdrl local_planner_with_localmap.launch
```

- /scan (sensor_msgs::LaserScan)
```
$ roslaunch mpdrl local_planner.launch
```

- /velodyne_obstacles (sensor_msgs::PointCloud2)
```
$ roslaunch mpdrl local_planner_with_pointcloud.launch
```

## published topics
- /cmd_vel(geometry_msgs::Twist)

## subscribed topics
- /local_goal(geometry_msgs::PoseStamped)

- /local_map(nav_msgs::OccupancyGrid)
- /scan (sensor_msgs::LaserScan)
- /velodyne_obstacles (sensor_msgs::PointCloud2)
