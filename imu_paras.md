# IMU Parameter Calibration
### Allan Variance
- [Implementation in C++](https://github.com/SiyuChen1/allan_variance_ros)
- [Detailed explanation of determining allan variance](https://de.mathworks.com/help/nav/ug/inertial-sensor-noise-analysis-using-allan-variance.html)
- [Another more detailed explanation](https://home.engineering.iastate.edu/shermanp/AERE432/lectures/Rate%20Gyros/14-xvagne04.pdf)
- [Additional resource](https://mwrona.com/posts/gyro-noise-analysis/)
### Determined by Dataset `rawimu_2023-11-24_09-28-32.log`
|                       Type                       | Datasheet | Allan-Variance |
|:------------------------------------------------:|:---------:|:--------------:|
|        Gyroscope Bias Instability (rad/s)        |  2.18e-6  |     6.98e-7    |
|       Gyroscope Noise Density (rad/sqrt(s))      |  1.75e-5  |     3.98e-5    |
|      Gyroscope Rate Random Walk (rad/s^1.5)      |           |     1.71e-7    |
|      Accelerometer Bias Instability (m/s^2)      |  7.35e-4  |      1e-4      |
|       Accelerometer Noise Density (m/s^1.5)      |    1e-3   |     1.76e-3    |
| Accelerometer Acceleration Random Walk (m/s^2.5) |           |     1.56e-5    |