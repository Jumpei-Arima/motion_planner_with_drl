# motion_planner_with_drl

## prerequisite
Please install tensorflow(cpu-version)
```
pip install tensorflow
```

# How to use
```
rosrun motion_planner_with_drl localmap_scan
rosrun motion_planner_with_drl motion_planner_drl.py
```

## published topics
- /cmd_vel(geometry_msgs::Twist)

## subscribed topics
- /local_map(nav_msgs::OccupancyGrid)
- /direction/relative(geometry_msgs::PoseStamped)
