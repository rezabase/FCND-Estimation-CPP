# FCND-Estimation-CPP
Udacity Flying Car - Estimation Assignment
Date: 01 Oct 2018


# Implement Estimator


## Determine the standard deviation of the measurement noise of both GPS X data and Accelerometer X data

Requirements: The calculated standard deviation should correctly capture ~68% of the sensor measurements. Your writeup should describe the method used for determining the standard deviation given the simulated sensor measurements.





## Implement a better rate gyro attitude integration scheme in the UpdateFromIMU() function

Requirements: The improved integration scheme should result in an attitude estimator of < 0.1 rad for each of the Euler angles for a duration of at least 3 seconds during the simulation. The integration scheme should use quaternions to improve performance over the current simple integration scheme.




## Implement all of the elements of the prediction step for the estimator

Requirements: The prediction step should include the state update element (PredictState() function), a correct calculation of the Rgb prime matrix, and a proper update of the state covariance. The acceleration should be accounted for as a command in the calculation of gPrime. The covariance update should follow the classic EKF update equation.


## Implement the magnetometer update

Requirements: The update should properly include the magnetometer data into the state. Note that the solution should make sure to correctly measure the angle error between the current state and the magnetometer value (error should be the short way around, not the long way).



## Implement the GPS update

Requirements: The estimator should correctly incorporate the GPS information to update the current state estimate.



# Flight Evaluation

## Meet the performance criteria of each step

Requirements: For each step of the project, the final estimator should be able to successfully meet the performance criteria with the controller provided. The estimator's parameters should be properly adjusted to satisfy each of the performance criteria elements.



## De-tune your controller to successfully fly the final desired box trajectory with your estimator and realistic sensors

Requirements: The controller developed in the previous project should be de-tuned to successfully meet the performance criteria of the final scenario (<1m error for entire box flight).




