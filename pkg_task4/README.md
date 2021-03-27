## Task 4 readme file

---
### Problem Statement

**Objective**: 
The objective of this task is to sort any nine packages out of twelve packages in any order as quickly as possible. To achieve this team needs to do the following things.

1. Identify the colour of the packages on shelf using Camera#1. Either colour detection or QR decoding or combination of both can be used here. NOTE: Hard-coding the colour of the packages is not allowed.

2. UR5#1 would have to pick the packages from the shelf and place it on the conveyor belt.

3. Conveyor belt needs to take the packages to UR5#2.

4. UR5#2 then needs to sort the packages based on the colour of the package identified by Camera#1. For eg. Red Package should go in the Red-Bin and so on.


* **Collision Avoidance**
While sorting the packages we need to make sure that the UR5 Arms (UR5#1 and UR5#2) are not colliding with the conveyor belt, packages on conveyor belt, with the shelf, packages in the shelf, with the bins or with itself. 
Once a package is picked we need to make sure that the package is also not colliding with anything.
You can use the concepts of Task-2 and Task-3 to implement collision avoidance.


* **Cameras**
In this task there are two logical cameras and one 2D camera.
The role of the 2D Camera is to use Computer Vision techniques to identify the packages in the shelf compartments.
The configuration of the packages is given as follows:
```shell
packagen00 - first package first row.
packagen01- second package first row.
packagen02 - third package first row.
packagen10 - first package second row.
packagen11 - second package second row.
packagen12 - third package second row.
packagen20 - first package third row.
packagen21 - second package third row.
packagen22 - third package third row.
packagen30 - first package fourth row.
packagen31 - second package fourth row.
packagen32 - third package fourth row.
```

* **TF**
In this task do not rely on TF values as they can fluctuate since we are using two UR5 arms. We would suggest to use Logical Camera#2 feed to manually calculate the translations. You can do this by using the concepts of TF which you learned in Task-3.

* **Simulation Time**
In this task simulation time will be considered for grading. So, we must make sure to keep the simulation time as low as possible by quickly sorting the packages.

**ROS Packages required:**

1. pkg_moveit_ur5_1 : we will have to generate this package using MoveIt! Setup Assistant which will configure MoveIt! for the UR5#1. For this use ur5_1.urdf in pkg_vb_sim/urdf.

2. pkg_moveit_ur5_2 : we will have to generate this package using MoveIt! Setup Assistant which will configure MoveIt! for the UR5#2. For this use ur5_2.urdf in pkg_vb_sim/urdf.

3. pkg_task4: This is the ROS package in which we are suppose to implement this task.

4. Vargi Bots Simulation Packages: These packages will have the simulation environment needed for this task.

### Launching this task solution

```shell
roslaunch pkg_task4 task4_solution.launch
```

### Output Video

YouTube Video Link :- https://youtu.be/uUPZPef96B4