### HOWTO

**Notes for the users of the FXIMU**

#### Prerequisites

sudo apt install ros-melodic-rviz-imu-plugin  
sudo apt install socat

#### Load parameters 

```
rosparam load config/fximu_params.yaml
```

>Notice: roscore must be running


#### Run rosserial

```
rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=230400
```

#### Run static transform publisher for rviz

```
rosrun tf static_transform_publisher 0.0 0.0 0.0 0.0 0.0 0.0 map base_imu_link 100 
```

#### Run RVIZ

Note that rviz imu plugin is required to visualize imu data. Click Add on rviz, then select rviz\_imu\_plugin then type 'imu/data' as topic in the visualization window.

#### Launch files

Use `fx.launch` to load parameters, and launch fximu. Use `fx_viz.launch` to launch fximu and RVIZ for testing.


#### View IMU data

```
rostopic echo /imu/data
```

#### View magnetometer data

```
rostopic echo /imu/mag
```

#### Measure frequency of output data

```
rostopic hz /imu/data
```

Or
  
```
rostopic hz /imu/mag
```