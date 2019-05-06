# Visual_Tracking_for_Corobots

1. First Initialize ROS
```
roscore
```

2. Start obstacle avoidance 
```
rosrun Visual_Tracking_for_Corobots Obstacle_Avoidance.py
```

3.  Start tracker \\
Valid Trackers \\
TLD, KCF, MOSSE, MIL or MEDIANFLOW
```
rosrun Visual_Tracking_for_Corobots Tracker.py "Tracker"
```

