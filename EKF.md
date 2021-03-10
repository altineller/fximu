### EKF

#### Robot Localization

`robot_localization` is a ROS package, that contains a generalized form of EKF, that can be used for any number of sensors, and inputs. In this application the data from IMU sensor is fused with data from odometry sensor, to  determine the robots position in 2D space.  

However, there are many diffuculties in configuring and tuning an IMU sensor to work with `robot_localization` package.

The `FXIMU` sensor, out-of-the-box, works with `robot_localization` package.

#### Prerequisites

```
sudo apt install ros-noetic-robot-localization -y
```

#### Configuration

In order for an IMU sensor to feed data to `robot_localization`:

1. Sensor must output data in physical units, and with correct magnitudes.
2. `world_frame` for orientation data must be in `ENU` format.
3. Covariances from all sensors must be correctly setup.

`FXIMU` does output in physical units. `world_frame` can be easily setup to `ENU` from parameters, and the covariances are output by the unit. These factors contribute to `FXIMU` being easily setup with `robot_localization` package.

>Notice: `FXIMU` internally runs a complementary ekf filter to find orientation. The EKF mentioned in this documentation, is run by `robot_localization` package, and it is a different filter.

Configuration file for `robot_localization` package is available at: [ekf_template](https://gitlab.com/ROSRider/rosrider/-/raw/master/config/ekf_template.yaml)

explain launch

explain input output

explain using /odometry/filtered

[TODO: covariances used]

