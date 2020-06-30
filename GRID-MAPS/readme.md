# GRID-MAPS
## Matlab libraries that implements the occgrid implementation described in:

1. Calle, I. "An efficient MATLAB implementation of grid maps and laser range finder likelihood models for the autonomous navigation of mobile robots ", to appear.
2. S. Thrun, W. Burgard, and D. Fox. Probabilistic Robotics, MIT-Press, 2005.


In order to be able to read the YAML files we use the library "yamlmatlab":
https://code.google.com/archive/p/yamlmatlab/downloads

---
### This library (the class 'occ_grid') has the following main features

  a. It is able to read and write the YAML files that are used by the Robot Operating System (ROS).
![Alt text](grid_map.jpg?raw=true "Title")

  b. It is able to compute the matrix of minimum distances for all the grid cells.
![Alt text](min_distances.jpg?raw=true "Title")

  c. It is able to compute the grid cells that correspond to the robot body (For mapping and obstacle avoidance purposes)
![Alt text](robot_space.jpg?raw=true "Title")
