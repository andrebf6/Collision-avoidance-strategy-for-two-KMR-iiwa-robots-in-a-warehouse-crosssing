# Collision-avoidance-strategy-for-two-KMR-iiwa-robots-in-a-warehouse-crosssing

## 1. Description
Repository containing software developed by Andrea Bravo Forn, as part of a final degree project in engineering physics performed at Federico II University (Naples, June 2021).

 <img align="left" width="300" height="200" src="https://user-images.githubusercontent.com/81975803/123519429-8b509d80-d6ab-11eb-9c3a-39ad059278b4.jpg">
 
The aim of this project was to design and implement in ROS a collision avoidance strategy for two KMR iiwa robots naviganting in a warehouse environment. 
To do so, the KMR iiwa robot model developed by Charlotte Heggem, Nina Marie Wahl and Morten M. Dahl (as a specialization project in Robotics & Automation at NTNU) has been used. The developed code is tested in Gazebo.

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

 This package handles navigation of the KMR iiwa robot. It contains the collsion avoidance algorithms: CA\_kuka (with communication) and CAE\_kuka (without communication).
  
 -  kmr_simulation

 This package sets up the simulation environment in Gazebo.
  
  ## 3. Run

In a terminal source the ROS workspace and run:
```
    ros2 launch kmr_model multi_spawn_robot_launch.py
```
To set up simulations in scenarios 1,2,3,4 and 6, open the multi_spawn_robot_launch.py file and set the adecuate initial position for robot 1 and robot 1. Robot 1's yaw needs to be yaw1=-1.57079633, while robot 2's yaw need to be yaw2=0.0. Open a two new terminals and souce the ros workspace in each of them. 

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

To set up simulations in scenario 5, open the multi_spawn_robot_launch.py file and set the adecuate initial position for robot 1 and robot 1. Robot 1's yaw needs to be yaw1=3.14159265, while robot 2's yaw need to be yaw2=0.0. Open a two new terminals and souce the ros workspace in each of them. 

If the algorithm with communication wants to be tested, in one terminal run:
```
    ros2 run kmr_navigation CA_kuka1_H.py
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

To change the simulation environment, open the gazebo.launch.py file, and change the variable word to the desired environment model (one of the SDF world models contained in the folder world).

To change the launched robot model, open the xml file of the launched simulation environment, and change the spawned robot model.
 
   
  
