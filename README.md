# raw-gnss-fusion

*Author: Jonas Beuchert*

This repository might acompany a future publication where we present an appraoch to fuse raw GNSS data with other sensing modalities using a factor graph. The goal is to localize a robot in the global Earth frame without drift.

**Table of Contents**

1. [Demo code for our carrier-phase factors](#demo-code-for-our-carrier-phase-factors)

2. [Instructions how to use our public robot dataset with GNSS, IMU, an lidar data](#instructions-how-to-use-our-public-robot-dataset-with-gnss-imu-and-lidar-data)

3. [Results of our method on various datasets](#results-of-our-method-on-various-datasets)

# Demo code for our carrier-phase factors

*TODO.*

# Instructions how to use our public robot dataset with GNSS, IMU, and lidar data

[Link to dataset (rosbags).]()

Setup:
* Platform: BostonDynamics Spot
* Lidar: HESAI Pandar XT
* IMU: Sevensense Alphasense
* GNSS receiver: u-blox C099-F9P
* GNSS antenna: u-blox ANN-MB-00

Recording rates:
* Lidar: 10 Hz
* IMU: 200 Hz
* GNSS: 5 Hz

Recording software:
* Operating System: Ubuntu 20.04.5 LTS
* IMU driver: alphasense_driver_ros
* GNSS driver: [ublox_driver](https://github.com/ori-drs/ublox_driver)

ROS topics:
* IMU: `/alphasense_driver_ros/imu` of type `sensor_msgs/Imu`
* Lidar: `/hesai/pandar` of type `sensor_msgs/PointCloud2`
* Raw GNSS: `/ublox_driver/range_meas` of type `gnss_comm/GnssMeasMsg`
* GNSS on-board fixes (DGNSS/RTK): `/ublox_driver/receiver_lla` of type `sensor_msgs/NavSatFix` or `/ublox_driver/receiver_pvt` of type `gnss_comm/GnssPVTSolnMsg`

Notes:
* IMU and lidar have a timeshift of 9.062 s w.r.t. the GNSS.
* The GNSS driver has a bug, it reported Galileo signals at frquency xxx, which should be at frequency xxx.

# Results of our method on various datasets

* Trajectory of a car in Hong Kong estimated with our algorithm fusing inertial and raw GNSS measurements (red) in comparison to RTK (ground truth, blue): [online map](https://users.ox.ac.uk/~kell5462/hong-kong.html), [file](hong-kong.html). Raw data from [GVINS Dataset](https://github.com/HKUST-Aerial-Robotics/GVINS-Dataset).
![Car in Hong Kong](hong-kong.png)

* Trajectory of a quadruped robot in the Bagley Wood estimated using inertial, raw GNSS, and lidar data (red) in comparison to RTK (ground truth, blue) and single GNSS fixes (orange): [online map](https://users.ox.ac.uk/~kell5462/bagley.html), [file](bagley.html).
![Quadruped in the Bagley Woods](bagley.png)

* Trajectory of a hand-held GNSS receiver in Oxford estimated using carrier phases only: [online map](https://users.ox.ac.uk/~kell5462/nhm.html), [file](nhm.html).
![Hand-held GNSS receiver in Oxford](nhm.png)

*More TODO.*
