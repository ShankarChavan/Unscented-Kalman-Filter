# Unscented Kalman Filter Project 
Self-Driving Car Engineer Nanodegree Program

In this project objective was to utilize an Unscented Kalman Filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. This project requires obtaining RMSE values that are lower than the tolerance.

---

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./UnscentedKF` Previous versions use i/o from text files.  The current state uses i/o
from the simulator.


We can observe in the below images that RMSE values are improved in UKF compare to EKF for Vx and Vy. 

## Output from code execution of UKF
![UKF Output](https://raw.github.com/ShankarChavan/Unscented-Kalman-Filter/master/img/UnscentedKF_result.png)

## Output from code execution of EKF
![EKF Output](https://raw.github.com/ShankarChavan/Unscented-Kalman-Filter/master/img/output1.png)



## NIS values

There is notebook to visualize below NIS plot in the repository.

I tried with different values for `std_a_` and `std_yawdd_`and finally settled for 1 and 0.3 values  

![NIS_Radar](https://raw.github.com/ShankarChavan/Unscented-Kalman-Filter/master/img/NIS_Radar.png)

Radar NIS values seems to be under 7.8 values 

![NIS_Laser](https://raw.github.com/ShankarChavan/Unscented-Kalman-Filter/master/img/NIS_Laser.png)



