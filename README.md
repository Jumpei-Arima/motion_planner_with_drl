# motion_planner_with_drl

## prerequisite
Please install tensorflow(cpu-version)
```
pip install tensorflow
```

# How to use
```
roslaunch motion_planner_with_drl local_planner.launch
```

## published topics
- /cmd_vel(geometry_msgs::Twist)

## subscribed topics
- /local_map(nav_msgs::OccupancyGrid)
- /direction/relative(geometry_msgs::PoseStamped)
