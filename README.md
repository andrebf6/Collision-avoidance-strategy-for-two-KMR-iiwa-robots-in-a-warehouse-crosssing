# Collision avoidance strategy for two KMR iiwa robots in a warehouse crossing

## 1. Description
Repository containing the software developed by Andrea Bravo Forn, as part of a bachelor degree thesis in Engineering Physics performed at the University of Naples Federico II (June 2021).

 <img align="left" width="280" height="170" src="https://user-images.githubusercontent.com/81975803/123519429-8b509d80-d6ab-11eb-9c3a-39ad059278b4.jpg">
 
This thesis on AGV fleed management lays within the framework of the European project ICOSAF. Its aim was to design, implement and test a distributed collision avoidance strategy for two KMR iiwa robots operating in a known, grid-like structured, smart warehouse. 

To test the developed strategy in a very realistic setting, ROS and Gazebo were used. To do so, the KMR iiwa ROS model developed by Charlotte Heggem, Nina Marie Wahl and Morten M. Dahl (as a specialization project in Robotics & Automation at NTNU) has been taken as an starting point. 

The present repository contains the implemented collision avoidance startegy for two KMR iiwa robots in a warehouse crossing.
In the repository andrebf6/Drive-a-single-KMR-iiwa, a navigation algorithm to drive a single KMR iiwa robot in a warehouse environment can be found.



System requirements:

 -  Ubuntu 18.04.3
 -  Python 3.6.9
 -  ROS2 Foxy Fitzroy
 -  Gazebo 11.0.0

Required ROS Packages:

  - Gazebo packages
  
  ## 2. Guide
  The repository contains the following packages:
  
   -  kmr_model
  
  This package contains an SDF model of the KMR robot. It is in charge of spawnning two robots in Gazebo when starting a simulation.
  
  -  kmr_navigation

 This package handles the navigation of the KMR iiwa robot. It contains the collsion avoidance algorithms: CA\_kuka (with communication) and CAE\_kuka (without communication).
  
 -  kmr_simulation

 This package sets up the simulation environment in Gazebo.
  
  ## 3. Run

In a terminal source the ROS workspace and run:
```
    ros2 launch kmr_simulation multi_kuka_launch.py
```
To set up simulations in scenarios 1,2,3,4 and 6, open the multi_spawn_robot_launch.py file (in the kmr_model package) and set the adecuate initial position for robot 1 and robot 2. Robot 1's yaw needs to be yaw1=-1.57079633, while robot 2's yaw needs to be yaw2=0.0. Open two new terminals and souce the ROS workspace in each of them. 

If the algorithm with communication wants to be tested, in one terminal run:
```
    ros2 run kmr_navigation CA_kuka1.py
```
and and in the other:
```
    ros2 run kmr_navigation CA_kuka2.py
```
If the algorithm without communication wants to be tested, in one terminal run:
```
    ros2 run kmr_navigation CAE_kuka1.py
```
and and in the other:
```
    ros2 run kmr_navigation CAE_kuka2.py
```

To set up simulations in scenario 5, open the multi_spawn_robot_launch.py file (in the kmr_model package) and set the adecuate initial position for robot 1 and robot 2. Robot 1's yaw needs to be yaw1=3.14159265, while robot 2's yaw needs to be yaw2=0.0. Open two new terminals and souce the ROS workspace in each of them. 

If the algorithm with communication wants to be tested, in one terminal run:
```
    ros2 run kmr_navigation CA_kuka1_EFH.py
```
and and in the other:
```
    ros2 run kmr_navigation CA_kuka2.py
```
If the algorithm without communication wants to be tested, in one terminal run:
```
    ros2 run kmr_navigation CAE_kuka1_EFH.py
```
and and in the other:
```
    ros2 run kmr_navigation CAE_kuka2.py
```

To change the simulation environment, open the start_world_launch.py file, and change the variable word to the desired environment model (one of the SDF world models contained in the folder world of the kmr_simulation package).
 
   
  
