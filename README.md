# ObjectOrientationEstimationSystem
This is a result after I investigated on many pieces of related published paper. It includes Matlab modeling and simulating on KF (Kalman Filter), EKF (Extended Kalman Filter) and other optimization approaches in the object-orientation-estimation system.

## Input
* Accelerometer raw data
* Gyroscope raw data
* Magnetometer raw data

## Output
* Estimated Euler angles
* Estimated Quaternion set

## Approaches
### Tilt-compensation Method  
![photo](https://github.com/jimmyvo2410/ObjectOrientationEstimationSystem/blob/master/Matlab/pic/tilt-2.jpg)

### Gauss Newton Method 
![photo](https://github.com/jimmyvo2410/ObjectOrientationEstimationSystem/blob/master/Matlab/pic/GN-2.jpg)

### AHRS
![photo](https://github.com/jimmyvo2410/ObjectOrientationEstimationSystem/blob/master/Matlab/pic/ahrs-2.jpg)

### Quaternion based – gyro bias – EKF 
![photo](https://github.com/jimmyvo2410/ObjectOrientationEstimationSystem/blob/master/Matlab/pic/EKFbias-tilt-2.jpg)

### Quaternion based – gyro rate – EKF
![photo](https://github.com/jimmyvo2410/ObjectOrientationEstimationSystem/blob/master/Matlab/pic/EKFgyro-tilt-2.jpg)

### DCM based KF 
![photo](https://github.com/jimmyvo2410/ObjectOrientationEstimationSystem/blob/master/Matlab/pic/KFdcm-2.jpg)

