# Project: Building a Controller

[![Estimation Video](http://img.youtube.com/vi/ZP4N_h6ZpKE/0.jpg)](http://www.youtube.com/watch?v=ZP4N_h6ZpKE)


## 1. STD of the measurement noise of GPS.X and IMU.X

I used `numpy.std` to calculate std for both them in this [notebook](https://github.com/dosht/FCND-Estimation-CPP/blob/master/STD.ipynb)


```python
import numpy as np
```
### Standard deviation of IMU Accelerometer X signals

```python
imu = np.genfromtxt("config/log/Graph2.txt", delimiter=",", skip_header=True)
np.std(imu[:,1])
```
    0.51242617825710723

### Standerd deviation of GPS X data

```python
gps = np.genfromtxt("config/log/Graph1.txt", delimiter=",", skip_header=True)
np.std(gps[:,1])
```
    0.71193945032223782

## 2. Implement a better rate gyro attitude integration scheme in the UpdateFromIMU() function
In this step, I implemented the complementary-filter to estimate pitch and roll by combining the sensor measurements from the Gyro and the accelerometer.
First, I calculated the `qt` as the quaternion that consists of the attitude in body frame by converting from Euler angles (pitch, roll, yaw) because we want to implement a non-linear complementary-filter.
Then, calculated the predicted quaternion `qtBar` by integrating the measured body rate from IMU multiplied by the IMU time constant `dtIMU`.
Finally, applying the following equation (which was already implemented) to combine in messurement from Gyro and the acceloremeter and using `attitudeTau` to tune the contribution of Gyro and acceloremeter in which `attitudeTau / (attitudeTau + dtIMU) + dtIMU / (attitudeTau + dtIMU` sum to 1.

![UpdateFromIMU equations 1](/images/UpdateFromIMU-equation1.png)

![UpdateFromIMU equations 2](/images/UpdateFromIMU-equation2.png)

![UpdateFromIMU equations 3](/images/UpdateFromIMU-equation3.png)


## 3. Implement all of the elements of the prediction step for the estimator
This is the "Predict" step in EKF. Our state vector in EKF is only `[x, y, z, xDot, yDot, zDot, psi]`. (Note update `theta` and `phi` in `UpdateFromIMU` method). I needed here to implement 3 methods:
1. `PredictState` that calculates the new state based only on the current state and the sensor data (accelerometer). It's a straightforward integration: position = position + velocity * time and velocity = velocity + acceleration * time. There are only 2 tricks, we use method: `attitude.Rotate_BtoI(accel)` to convert the accelerometer measurements from body frame to inertial frame and the second one is we subtract gravity from "Z" acceleration because the accelerometer Z always measure the acceleration of gravity.

![PredictState equations](/images/PredictState-equation.png)

2. `GetRbgPrime` that returns the rotation matrix as follows:

![GetRbgPrime equations](/images/GetRbgPrime-equation.png)

3. `Predict`, the main method in the transition model that reads the `RgbPrime`, the previous state and the accelerometer measurement to predict the covariance.

![Predict equations](/images/Predict-equation.png)

## 4. Implement the magnetometer update
In this step, we read the magnetometer measurements for yaw angle, and I implemented the derivative as described in "Estimation for Quadratus" document.

Derivitave equation:

![Magnetometer equations](/images/Magnetometer-equation.png)

## 5. Implement the GPS update
In this step, we read GPS measurement of position and velocity, and I implemented the derivative as described in "Estimation for Quadratus" document.

Derivitave equation:

![GPS equations](/images/GPS-equation.png)

## 6. Use my code from the [FCND-Control-CPP](https://github.com/dosht/FCND-Controls-CPP) project
In that final step, I replaced the existing control implementation with my code from the previous project and I noticed the reactions of the drone is too aggressive and crashes after a while, so I re-tuned the control params as suggested by reducing the position and velocity gain parameters while keeping the percentage almost the same between `(kpPosXY, kpPosXY)` and `(kpPosZ, kpVelZ)` until the GPS-Update scenario worked as expected. I didn't need to change `KiPosZ`.
