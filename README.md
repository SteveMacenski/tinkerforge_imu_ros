# tinkerforge_imu_ros
ROS drivers for Tinkerforge V2 IMU

Basic ROS driver for the [Tinkerforge IMU V2 Brick](https://www.tinkerforge.com/en/doc/Hardware/Bricks/IMU_V2_Brick.html). This should work with all ROS1 distributions Indigo through Noetic.

This ROS driver contains the following:
- Install script to automate install and launching of the tinkerforge debian packaged daemon
- Launch file for your IMU Brick requiring your Brick's UID as an argument
- IMU driver producing an orientation vector and covariance matrix **only**. PRs to add additional IMU message information is encouraged.
