## Task 2 readme file

---
### Problem Statement
The objective of this task is to implement pick and place using one UR5 (UR5#1) in Gazebo.

The UR5 (UR5#1) needs to pick the blue package from the shelf and place it in the bin next to it.
While doing this we need to make sure that the UR5 (UR5#1) is not colliding with the shelf, the bin or with itself.
After the blue package is picked we need to make sure that the package also does not collide with the shelf.
To do this collision avoidance we can add the shelf and the box in the MoveIt! Planning Scene for the Planner.

we will need the following packages for this task:

1. pkg_moveit_ur5_1 : The we will have to generate this package using MoveIt! Setup Assistant which will configure MoveIt! for the UR5 (UR5#1).

2. pkg_task2: This is the package in which you will implement the pick and place and the package we need to submit.

3. Vargi Bots Simulation Packages: These packages will have the simulation environment needed for this task.

### Launching this task solution

```shell
roslaunch pkg_task2 task2.launch
```
### Output Video

YouTube Video Link :- https://youtu.be/211rY7t1saw
