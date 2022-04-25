# Turtlebot3 A star path planner

## Dependencies
- python 3.6 or above
- numpy
- matplotlib
- opencv

## How to run?

Open the terminal at the location of ```planner.py``` file, and enter
This will use default start pose (1,1,0) and goal pose (9,9,0)

```
python3 planner.py
```

To provide arguments use following syntax (units in meters)

```
# python3 planner.py start_x, start_y, start_theta, goal_x, goal_y, rpm1, rpm2, clearance
python3 planner.py 1.0 1.0 0.0 9.0 9.0 5 10 0.1     # example 1
python3 planner.py 1.0 1.0 1.56 9.0 9.0 5 10 0.1    # example 2
```

Running this command will open a window with available region and obstacles. Animation will show the explloration of map to reach goal location and found path.

## Misc.

- Implementation takes less than 1.0 sec to find path between 2 farthest points in the map.