DISCONTINUATION OF PROJECT.

This project will no longer be maintained by Intel.

Intel has ceased development and contributions including, but not limited to, maintenance, bug fixes, new releases, or updates, to this project. 

Intel no longer accepts patches to this project.

If you have an ongoing need to use this project, are interested in independently developing it, or would like to maintain patches for the open source software community, please create your own fork of this project. 
# imu_driver
ROS driver for IMU sensor devices

## Prerequisite
 * Ubuntu 16.04
 * ROS Kinetic
 
## How to Fetch
` $ cd ~/catkin_ws/src`

`$ git clone https://github.intel.com/drones/imu_driver.git`

## How to Build
`$ cd ~/catkin_ws`

`$ catkin build imu_driver`

## How to Run
`$ rosrun imu_driver imu_driver_node`

## How to Test
1. Run following command and whether IMU ROS topics `imu_driver/*` are listed

   `$ rostopic list`

2. Run following command and check whether all GTests are passed

   `$ catkin run_tests imu_driver`
   
   `$ catkin_test_results imu_driver`

###### *Any security issue should be reported using process at https://01.org/security*
