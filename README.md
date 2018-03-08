# Unscented Kalman Filter Project 
Self-Driving Car Engineer Nanodegree Program

In this project utilize an Unscented Kalman Filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. Passing the project requires obtaining RMSE values that are lower than the tolerance.


---

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./UnscentedKF` Previous versions use i/o from text files.  The current state uses i/o
from the simulator.

## Output from code execution
![UKF Output](https://raw.github.com/ShankarChavan/Unscented-Kalman-Filter/img/UnscentedKF_result.png)

We can observe in the above image that RMSE values are below the tolearnce level.

## NIS values

![NIS_Laser](https://raw.github.com/ShankarChavan/Unscented-Kalman-Filter/img/NIS_Laser.png)

![NIS_Radar](https://raw.github.com/ShankarChavan/Unscented-Kalman-Filter/img/NIS_Radar.png)

