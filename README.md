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
