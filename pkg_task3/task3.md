## Task 3 readme file

---
### Problem Statement
Objective: The objective of this task is to sort the packages as quickly as possible using the same UR5 (UR5#1) that was used in Task-2.



1. Sorting

The UR5 (UR5#1) needs to pick the packages from the conveyor belt and place it in the correct bin. Red-Package will go in Red Bin, Blue-Package in Blue Bin and Green-Package in Green Bin.

2. Collision Avoidance

While sorting the packages the we need to make sure that the UR5 (UR5#1) is not colliding with the conveyor belt, packages on conveyor belt, the bins or with itself. Once a package is picked the team needs to make sure that the package is also not colliding with anything.


1. Logical Camera and Conveyor Belt

We need to use the feed from Logical Camera in this task to detect packages.
We would also have to control the conveyor belt in order to make the packages reach the UR5 (UR5#1).

4. TF
   
Teams may have to use TF of the package and End-Effector to make the EE go to the package.

5. Simulation Time

In this task simulation time will be considered for grading. So, the teams must make sure to keep the simulation time as low as possible by quickly sorting the package.

Strategies to decrease Simulation Time

The team can operate the conveyor belt at maximum speed so that packages reach the UR5 faster.
The team can make the UR5 pick the packages from a moving conveyor belt.
Or any other such strategies.

<br>

ROS Packages required:

1. pkg_moveit_ur5_1 : The team will have to generate this package using MoveIt! Setup Assistant which will configure MoveIt! for the UR5 (UR5#1). You may use the same pkg_moveit_ur5_1 which you have generated for Task-2.

2. pkg_task3: This is the ROS package in which the team is suppose to implement this task.

3. Vargi Bots Simulation Packages: These packages will have the simulation environment needed for this task.
<br>


### Output Video

YouTube Video Link :- https://youtu.be/bHFJ7d8q538
