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
In another terminal run below command and check if you can see the topics `imu_driver/*`

`$ rostopic list`
