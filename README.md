# Visual_Tracking_for_Corobots

1. First Initialize ROS<br />
```
roscore
```

2. Start obstacle avoidance <br />
```
rosrun Visual_Tracking_for_Corobots Obstacle_Avoidance.py
```

3.  Start tracker  <br />
Valid Trackers: TLD, KCF, MOSSE, MIL or MEDIANFLOW<br />
```
rosrun Visual_Tracking_for_Corobots Tracker.py "Tracker"
```

