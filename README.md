# motion_planner_with_drl

## prerequisite
if you have model learned by tensorflow.
Please install tensorflow(cpu-version)
```
pip install tensorflow
```

if you have model learned by pytorch
you need to convert your PyTorch Model to Torch Script by tracing.
[watch here](https://pytorch.org/tutorials/advanced/cpp_export.html)

# How to use
if you use tensorflow model -> run this launch
```
roslaunch motion_planner_with_drl local_planner.launch
```

if you use torch model -> run this launch
```
roslaunch motion_planner_with_drl demo.launch
```

input
- local_map
  you need to convert local mpa to scan by using [localmap2scan](https://github.com/Jumpei-Arima/motion_planner_with_drl/blob/master/src/localmap_scan.cpp).
- point cloud
  - you need to convert pointcloud to scan by using [pointcloud_to_laserscan](http://wiki.ros.org/pointcloud_to_laserscan) package.
- scan

## published topics
- /cmd_vel(geometry_msgs::Twist)

## subscribed topics
- /local_map(nav_msgs::OccupancyGrid)
- /direction/relative(geometry_msgs::PoseStamped)
