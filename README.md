# Visual_Tracking_for_Corobots

1. First Initialize ROS
```
roscore
```

2. Start obstacle avoidance 
```
rosrun Visual_Tracking_for_Corobots Obstacle_Avoidance.py
```

3.  Start tracker 
LAst Paramater can be TLD, KCF, MOSSE, MIL, MEDIANFLOW 
```
rosrun Visual_Tracking_for_Corobots Tracker.py TLD
```

