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

```
<launch>
  <node pkg="robot_localization" type="ekf_localization_node" name="ekf_se" clear_params="true">
    <rosparam command="load" file="$(find rosrider)/config/ekf_template.yaml" />
  </node>
</launch>

```

### Filtered Odometry

TODO: explain input output

TODO: explain using /odometry/filtered

TODO: explain how you need to rewire the goal controller with this.

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



