# e-Yantra 2020-21 Vargi - Bots

## Introduction to the Vargi Bots Theme
e-Yantra Robotics Competition features a theme called ‘Vargi-Bots’. Vargi is taken from a Sanskrit word, Vargikaran (वर्गीकरण) which means to separate objects based on their category. The theme is set in the abstraction of a warehouse management system designed in Gazebo, which is a 3D dynamic simulator used to efficiently simulate robots in complex environments.

The arena is an automated warehouse setting where essential packages are required to be sent out to different parts of a city. Since Industry 4.0 heavily focuses on automation here the warehouse will only consist of two industrial robotic arms which will be used by the teams. As the requirements are sent to the warehouse, one robotic arm will identify the packages from a shelf and place them on a conveyor belt and the other robotic arm at the end of the conveyor belt will pick these objects from the conveyor and place them into bins. Each bin represents a destination for the package. 


## Task wise Problem Statement and Outputs

[Task 0](https://github.com/jayBhagiya/e-Yantra-2020-21/tree/main/pkg_task0)<br>
[Task 1](https://github.com/jayBhagiya/e-Yantra-2020-21/tree/main/pkg_task1)<br>
[Task 2](https://github.com/jayBhagiya/e-Yantra-2020-21/tree/main/pkg_task2)<br>
[Task 3](https://github.com/jayBhagiya/e-Yantra-2020-21/tree/main/pkg_task3)<br>
[Task 4](https://github.com/jayBhagiya/e-Yantra-2020-21/tree/main/pkg_task4)


## How to Replicate this Project

It is Assumed that you have ROS Melodic,Gazebo 9.0 and Moveit pre-installed and configured properly 

Step 1 :- clone this repository inside the src folder of your desired workspace.

```shell
cd catkin_ws/src

git clone https://github.com/jayBhagiya/e-Yantra-2020-21.git
```

Step 2 :- Now build the workspace and source the setup.bash files
```shell
cd catkin_ws/

catkin build

source ~/catkin_ws/devel/setup.bash
```

Step 3 :- Running the Desired task on your machine ( launch file name may vary according to the pkg you are running see launch folder)

```shell
roslaunch pkg_task3 task3_solution.launch
```

Now you will see the simulation of UR5 robotic arm in Gazebo and Rviz.

Contributors 
JEEL CHATROLA <br/>
JAY BHAGIYA

