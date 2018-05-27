## 1. STD of the measurement noise of GPS.X and IMU.X

I used `numpy.std` to calculate std for both them in this [notebook](https://github.com/dosht/FCND-Estimation-CPP/blob/master/STD.ipynb)


```python
import numpy as np
```
### Standard deviation of IMU Accelerometer X singals

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
## 3. Implement all of the elements of the prediction step for the estimator
## 4. Implement the magnetometer update
## 5. Implement the GPS update
