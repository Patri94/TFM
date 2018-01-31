# TFM
Master Thesis
Author: Patricia Javierre 
This repository contains different ROS packages building localization algorithm for a mobile robot based on the MCL algorithm, as well as the files
needed to simulate the robot for test purposes.

#SIMULATION: model of Doris and environmnets.
  -world -> environmnets
  -xacro->  robot model
  -launch -> launch files for gazebo and robot state publisher.

#CONNECTON : interface between Doris and ROS.

#DETECTOR : detection of the spedific landmarks.
  -launch-> files for running the node. 
      - Flag simulation : 0 -> using the omnidirectional image.
                          1 -> combining information of three cameras.
#AMCL_DORIS : localization node.
  -pf -> treatment of particles.
  -sensors -> treatment of odometry, laser and camera.
  -amcl_node.cpp ->principal file.
  -maps-> visual maps.
  -examples -> launch files for simulation and real experiments
  
#NAVIGATION FILES : occupancy grid maps for the laser sensor. Necessary to run the map_server of ROS.
#GRAPHS: matlab code for representing errors using txt files.
  
