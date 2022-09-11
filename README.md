# raw-gnss-fusion

*Author: Jonas Beuchert*

This repository might acompany a future publication where we present an appraoch to fuse raw GNSS data with other sensing modalities using a factor graph. The goal is to localize a robot in the global Earth frame without drift.

**Table of Contents**

1. [Demo code for our carrier-phase factors](#a)

2. [Instructions how to use our public robot dataset with GNSS, IMU, an lidar data](#b)

3. [Results of our method on various datasets](#results-of-our-method-on-various-datasets)

# Results of our method on various datasets

* Trajectory of a car in Hong Kong estimated with our algorithm fusing inertial and raw GNSS measurements (red) in comparison to RTK (ground truth, blue). Data from [GVINS](https://github.com/HKUST-Aerial-Robotics/GVINS-Dataset): [link](), [file]().

* Trajectory of a quadruped robot in the Bagley Wood estimated using inertial, raw GNSS, and lidar data (red) in comparison to RTK (ground truth, blue) and single GNSS fixes (orange): [link](), [file]().

* Trajectory of a hand-held GNSS receiver in Oxford estimated using carrier phases only: [link](), [file]().
