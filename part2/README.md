# Turtlebot3 Gazebo simulation using A star path planner

## Dependencies
- python 3.6 or above
- numpy
- matplotlib
- opencv
- turtlebot3


## Install Turtlebot3
Install turtlebot3 packages using instructions
from [here](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/).

## How to run?

Copy project3 folder in your catkin workspace.
build the package using,
```
cd catkin_ws
export TURTLEBOT3_MODEL=burger
catkin build project3
source devel/setup.bash
```
Launch the nodes using following command
```
roslaunch project3 turtlebot3_map.launch
# or
roslaunch project3 turtlebot3_map.launch start_x:=1 start_y:=1 start_angle:=0 goal_x:=9 goal_y:=9
```

On launching the gazebo simulator, running node will first plan the path and then user can see the the robot moving toward goal location.

## Misc.

- Implementation takes less than 1.0 sec to find path between 2 farthest points in the map.