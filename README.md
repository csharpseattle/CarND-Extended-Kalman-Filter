# Extended Kalman Filter
Chris Sharp

Self-Driving Car Engineer Nanodegree Program

In this project a kalman filter, written in C++, was used to estimate the state of a moving object with noisy lidar and radar measurements.  The provided lidar and radar measurements are used to track the object's position and velocity as it moves around the sensor.

The project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases).


## uWebSockets

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems.

## Building

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ExtendedKF


## Files

- `main.cpp` - main entry point and websocket handlers.
- `FusionEKF.cpp, FusionEKF.h` - handles processing of measurements, management of matrices for LIDAR, RADAR.
- `kalman_filter.cpp, kalman_filter.h` - KalmanFilter class implementation.
- `tools.cpp, tools.h` - utilities for Jacobian matrix creation and calculation of Root Mean Squared Error (RMSE)
