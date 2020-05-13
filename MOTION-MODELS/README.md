# MOTION-MODELS
## Matlab libraries that implements the probabilistic motion models described in:

1. S. Thrun, W. Burgard, and D. Fox. Probabilistic Robotics, MIT-Press, 2005.
2. Calle, I. "Time-Invariant Gaussian Velocity Motion Models for Mobile Robots", 
   2019 IEEE XXVI International Conference on Electronics, Electrical Engineering and Computing (INTERCON).
   https://ieeexplore.ieee.org/document/8853606
3. Calle, I. "Improved Odometry Motion Models for Differential-like Mobile Robots", to appear.

---
All the implementation of the algorithms are inside de class "robot". So you just need to instantiate objects of this class to create "robots".
   - In the folder "BASICS" we see simple examples to use this class.
   - In the folder "VELOCITY" we have examples for [1] and [2]
   - In the folder "ODOMETRY" we have examples for [1] and [3]
   - To use this demos you must add to the Matlab path the folder TOOLS

---
Finally the functions "sample_motion_model_odometry", "sample_motion_model_velocity", "motion_model_odometry", and "motion_model_velocity" are high performance vectorized implementations of the sampling and density algorithms.
