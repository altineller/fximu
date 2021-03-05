### Quickstart

#### Prerequisites

```		
sudo apt install ros-noetic-rviz-imu-plugin -y  
sudo apt install socat -y  
```

#### FXIMU package

FXIMU does not need a driver, or additional software to connect to ROS, however launch and configuration files are presented in a package for convenience to the user. 

To get the fximu package:

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/altineller/fximu.git
catkin_make
source ~/catkin_ws/devel/setup.bash
```

Once you have the fximu package, go to launch directory and edit fximu.launch:

```
roscd fximu/launch
nano fximu.launch
```

In this file, we load two set of parameters, the first `fximu_params.yaml` file contains parameters about the operation of the device, and the numbered `fximu_params_000.yaml` file contains parameters about calibration of that specific device.

```
<launch>
    <rosparam command="load" file="$(find fximu)/config/fximu_params.yaml" />
    <rosparam command="load" file="$(find fximu)/config/fximu_params_000.yaml" />
    <include file="$(find fximu)/launch/fx.launch">
        <arg name="serial_device" value="/dev/ttyACM0"/>
    </include>
</launch>
```

Edit the `000` in `fximu_params_000.yaml` string, to match the serial_id of your FXIMU.

Edit the `ttyACM0` string at the end to match your serial port, and save file.

Before running the full demo, lets see if the connection works first. Open a terminal and launch `fximu.launch`

```
roscd fximu/launch
roslaunch fximu.launch
```  

You should see:

```console
process[serial_node_fximu-2]: started with pid [1652]
[INFO] [1614953635.669295]: ROS Serial Python Node
[INFO] [1614953635.672512]: Connecting to /dev/ttyACM0 at 230400 baud
[INFO] [1614953637.777565]: Requesting topics...
[INFO] [1614953637.839926]: Note: publish buffer size is 512 bytes
[INFO] [1614953637.841215]: Setup publisher on imu/data [sensor_msgs/Imu]
[INFO] [1614953637.844869]: Setup publisher on imu/mag [sensor_msgs/MagneticField]
[INFO] [1614953637.848249]: Setup publisher on imu/raw [std_msgs/Int16MultiArray]
[INFO] [1614953637.868916]: calibration_mode: 0
[INFO] [1614953637.882941]: FXIMU Parameters read from EEPROM
[INFO] [1614953637.897206]: Firmware Revision: 01MARCH2021
[INFO] [1614953637.899882]: sensor_read_rate: 400
[INFO] [1614953637.902283]: output_rate_divider: 8
[INFO] [1614953637.910633]: adaptive_gain: 1
[INFO] [1614953637.913407]: bias_estimation: 1
[INFO] [1614953637.925089]: gain_acc: 0.020
[INFO] [1614953637.929318]: gain_mag: 0.010
[INFO] [1614953637.934133]: bias_alpha: 0.100
[INFO] [1614953637.939420]: imu_frame_id: base_imu_link
[INFO] [1614953637.943528]: mag_frame_id: base_mag_link
[INFO] [1614953637.953971]: GFSR: 2
[INFO] [1614953637.958144]: AFSR: 1
[INFO] [1614953637.962534]: steady_limit: 32
[INFO] [1614953637.966951]: world_frame: 0
[INFO] [1614953637.971072]: use_mag: 1
[INFO] [1614953637.981785]: kAngularVelocityThreshold: 0.060
[INFO] [1614953637.986130]: kAccelerationThreshold: 0.250
[INFO] [1614953637.990327]: kDeltaAngularVelocityThreshold: 0.050
[INFO] [1614953637.995843]: mag_offsets: 19.190, -19.830, -67.510
[INFO] [1614953638.009829]: soft_iron_matrix[0]: 0.977, -0.034, -0.014
[INFO] [1614953638.023886]: soft_iron_matrix[1]: -0.034, 0.992, 0.020
```

The FXIMU comes calibrated, and the calibration values are read from device at bootup, when `calibration_mode=0`

If you connected successfully to the device, Press `CTRL-C` to stop, and this time run `roslaunch fxviz.launch` to run the full demo, which launches the serial connection, then launches RVIZ with custom configuration. Note that rviz imu plugin is required to visualize imu data.

![rviz](doc/rviz.png)
