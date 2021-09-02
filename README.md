# SLAM_Igluna
This repository contains all the work of Maximilien Dreier during his internship from November 2020 to July 2021 concerning Igluna 2021 Edition, in the team Corodro of SUPAERO.

The goal of the project was to developp the algorithm enabling autonomous exploration of a Rover and a flying drone in an unknown environment.
The work here focuses on the Simultaneous Localization and Mapping algorithms.
A T265 and a D435i camera were used as sensors.
They were used on the drone with the algorithm "octomap_server" to generate a 3D map of the environment.
Then, a sequence of created algorithms were used to transform that 3D map into a 2D occupancy grid map that can be used by the rover. Their order of utlization as well as their 
description can be read in the document "SLAM PROCEDURE" available in the repository.
(Algo conversion_extraction, denoising, grid_map_generator, grid_map_pcl_saver, map_pub.cpp, occupancy_saver, occupancymap_generator, outliers_removal,
pointcloud_saver, radius.cpp, read_pcd.cpp)

"ar_track_alvar" is the algorithm used to detect the AR tag on the ground. Those ar tag simulated our objectives on the field.
Algorithms were created to create a database of all the ar tags detected by that algorithm (both on rover and on the drone).
Also, those databases were used to compute the coordinate transformation between the rover and the drone, and thus creating a link between the two robotic systems.
(Algo ar_tracker_saver.cpp, ar_tf.cpp)

An algorithm was also created to generate the problem file HDLL for the task planner algorithm.
(Algo occupancymap_planner)

You'll find in this repository the Student documentation presenting my full work and its results (including the Final testing week at Esperce in late June 2021) as well as my
Master Thesis concerning this internship presenting my work and the theory behind the implemented algorithm (this thesis was due for End of April 2021 and mentions the work up to that date).

The ending video presenting our team work and the tests at Esperce is available here :
https://drive.google.com/file/d/1jlD5jmhru6JPV-2YkScTGlvmlNnWypEy/view


I (Maximilien Dreier), stay available for any question at maximilienlp@gmail.com 
