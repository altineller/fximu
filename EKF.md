### EKF

#### Prerequisites

```
sudo apt install ros-noetic-robot-localization -y
```

#### Robot Localization

`robot_localization` is a ROS package, that contains a generalized form of EKF, that can be used for any number of sensors, and inputs. In this application the data from IMU sensor is fused with data from odometry sensor, to  determine the robots position in 2D space.  

However, there are many diffuculties in configuring and tuning an IMU sensor to work with `robot_localization` package.

The `FXIMU` sensor, out-of-the-box, works with `robot_localization` package.

#### Configuration

In order for an IMU sensor to feed data to `robot_localization`:

1. Sensor must output data in physical units, and with correct magnitudes.
2. `world_frame` for orientation data must be in `ENU` format.
3. Covariances from all sensors must be correctly setup.

`FXIMU` does output in physical units. `world_frame` can be easily setup to `ENU` from parameters, and the covariances are output by the unit. These factors contribute to `FXIMU` being easily setup with `robot_localization` package.

>Notice: `FXIMU` internally runs a complementary ekf filter to find orientation. The EKF mentioned in this documentation, is run by `robot_localization` package, and it is a different filter.

Configuration file for `robot_localization` package is available at: [ekf_template](https://gitlab.com/ROSRider/rosrider/-/raw/master/config/ekf_template.yaml)

### Launching

Add the following to your robots launch file. Basically the following statement launches  `robot_localization`, with the `ekf_template.yaml` file as configuration.

```
<launch>
  <node pkg="robot_localization" type="ekf_localization_node" name="ekf_se" clear_params="true">
    <rosparam command="load" file="$(find rosrider)/config/ekf_template.yaml" />
  </node>
</launch>
```

### Filtered Odometry

The EKF filter with the configuration above listens for odometry and imu data, and fuses them together to obtained filtered odometry.

Filtered odometry is output at `/odometry/filtered` Any goal controller or move base should listen to `/odometry/filtered` for making calculations. 

The goal controller provided with `rosrider_diff_drive` looks for the `goal_controller_odom_topic` to listen for during operation. Launching the provided files with `robot_ekf.launch` will start with the robot, goal controller listening to `/odometry/filtered`

#### Covariances

Below is the relevant part in firmware code:

```
imu_msg.angular_velocity_covariance[0] = 0.02;
imu_msg.angular_velocity_covariance[1] = 0;
imu_msg.angular_velocity_covariance[2] = 0;
imu_msg.angular_velocity_covariance[3] = 0;
imu_msg.angular_velocity_covariance[4] = 0.02;
imu_msg.angular_velocity_covariance[5] = 0;
imu_msg.angular_velocity_covariance[6] = 0;
imu_msg.angular_velocity_covariance[7] = 0;
imu_msg.angular_velocity_covariance[8] = 0.02;

imu_msg.linear_acceleration_covariance[0] = 0.04;
imu_msg.linear_acceleration_covariance[1] = 0;
imu_msg.linear_acceleration_covariance[2] = 0;
imu_msg.linear_acceleration_covariance[3] = 0;
imu_msg.linear_acceleration_covariance[4] = 0.04;
imu_msg.linear_acceleration_covariance[5] = 0;
imu_msg.linear_acceleration_covariance[6] = 0;
imu_msg.linear_acceleration_covariance[7] = 0;
imu_msg.linear_acceleration_covariance[8] = 0.04;

imu_msg.orientation_covariance[0] = 0.0025;
imu_msg.orientation_covariance[1] = 0;
imu_msg.orientation_covariance[2] = 0;
imu_msg.orientation_covariance[3] = 0;
imu_msg.orientation_covariance[4] = 0.0025;
imu_msg.orientation_covariance[5] = 0;
imu_msg.orientation_covariance[6] = 0;
imu_msg.orientation_covariance[7] = 0;
imu_msg.orientation_covariance[8] = 0.0025;
```

#### References

Most of these instructions were made possible by: 

- [ros sensor fusion tutorial](https://github.com/methylDragon/ros-sensor-fusion-tutorial)

- [preparing sensor data](http://docs.ros.org/en/melodic/api/robot_localization/html/preparing_sensor_data.html)

#### Licenses

|![tr000003](https://raw.githubusercontent.com/rosrider/fximu_doc/main/img/TR000003.png)   |![license](https://raw.githubusercontent.com/ROSRider/fximu_doc/main/img/license.png)|
|----|----|

