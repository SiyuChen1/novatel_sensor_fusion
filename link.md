+ [Create a ROS2 package for Both Python and Cpp Nodes](https://roboticsbackend.com/ros2-package-for-both-python-and-cpp-nodes/)
+ [CPT7 and CPT7700 Installation and Operation User Manual](https://docs.novatel.com/OEM7/Content/PDFs/CPT7_Installation_Operation_Manual.pdf)
+ [The Continuous-Time Kalman Filter](https://webee.technion.ac.il/people/shimkin/Estimation09/ch5_cont.pdf)
+ [Quaternion kinematics for the error-state Kalman filter](https://www.iri.upc.edu/people/jsola/JoanSola/objectes/notes/kinematics.pdf)
+ [IMU and GPS Fusion for Inertial Navigation](https://de.mathworks.com/help/fusion/ug/imu-and-gps-fusion-for-inertial-navigation.html)
    - 22 States (quaternion 4 + position 3 + velocity 3 + angle bias 3 + velocity bias 3 + geomagnetic field vector 3 + magnetometer bias 3)
    - [Implementation of Filter](https://de.mathworks.com/help/fusion/ref/insfiltermarg.html)
+ [Pose Estimation From Asynchronous Sensors(IMU,GPS)](https://de.mathworks.com/help/fusion/ug/pose-estimation-from-asynchronous-sensors.html)
    - The filter uses a 28-element state vector to track the orientation quaternion, velocity, position, MARG sensor biases, and geomagnetic vector. The insfilterAsync object uses a continuous-discrete extended Kalman filter to estimate these quantities.
    - [Implementation of Filter](https://de.mathworks.com/help/nav/ref/insfilterasync.html)
    - Question is about the measurement model
