### Parameters

#### Calibration Mode

| Calibration Mode | Description            
| --- | ----------------------
| 0   | EEPROM defaults will be used. this is the default mode
| 1   | calibration mode. raw sensor valuew will be sent
| 2   | will use parameters from rosparam
| 3   | will use parameters from rosparam, and write them to EEPROM


To calibrate the device, set the calibration mode to `1`. See: [CALIBRATION](CALIBRATION.md)

After the calibration values are obtained, plug them into the `fximu_params.yaml` file, and set calibration mode to `2`.

If the results are good enough, set calibration mode to `3`. Reconnect to fximu. It will write the parameters to EEPROM.

Once parameters are written to EEPROM, you can set the calibration mode to `0`, and the device will use internal EEPROM values, and ignore the calibration values in the yaml file.

The device will report about the status of the calibration on the console output.

#### Parameter Table

| Parameter  | Description            
| --- | ----------------------
| sensor\_read\_rate    | Sensor read frequency. Can be: 50,100,200, or 400
| output\_rate\_divider | Output rate divider. Can be: 1,2,4,8,16
| adaptive_gain | Use adaptive gain
| bias\_estimation | Use bias estimation
| gain\_acc | Accelerometer gain for the complementary filter
| gain\_mag | Magnetometer gain for the complementary filter
| bias\_alpha | Bias estimation gain for the complementary filter
| gfsr | Gyro hardware sensitivity
| afsr | Accelerometer hardware sensitivity
| world\_frame | 0=NWU, 1=ENU. Notice only output orientation quaternion is transformed, not the sensor data
| use\_mag | Use magnetometer. For magnetically noisy environments, you can turn magnetometer off
| kAngularVelocityThreshold | Angular velocity threshold for steady state detection
| kAccelerationThreshold | Acceleration threshold for steady state detection
| kDeltaAngularVelocityThreshold | Delta Angular velocity threshold for steady state detection
| steady\_limit | After thresholds are not exceeded N times, device is on steady state, and green light is on. Valid values are 2 to 127
| mag\_offset\_x | Magnetometer offset x
| mag\_offset\_y | Magnetometer offset y
| mag\_offset\_z | Magnetometer offset z
| mag\_soft\_iron\_ix | Soft iron matrix ix
| mag\_soft\_iron\_iy | Soft iron matrix iy
| mag\_soft\_iron\_iz | Soft iron matrix iz
| mag\_soft\_iron\_jx | Soft iron matrix jx
| mag\_soft\_iron\_jy | Soft iron matrix jy
| mag\_soft\_iron\_jz | Soft iron matrix jz
| mag\_soft\_iron\_kx | Soft iron matrix kz
| mag\_soft\_iron\_ky | Soft iron matrix ky
| mag\_soft\_iron\_kz | Soft iron matrix kz
| imu\_frame\_id | `frame_id` for `Imu` message
| mag\_frame\_id | `frame_id` for `MagneticField` message

The output rate will equal `sensor_read_rate` / `output_rate_divider`.

Use a high sensor read rate, then divide it to get a useful output rate. The calculations are done at sensor read rate, which leads to filter converge faster.

The frame_id's for the respective `Imu` and `MagneticField` messages can be setup with the `imu_frame_id` and `mag_frame_id` parameters.

Parameters `adaptive_gain`,`bias_estimation`,`gain_acc`,`gain_mag`,`bias_alpha` are also explained in [http://wiki.ros.org/imu\_complementary\_filter](http://wiki.ros.org/imu_complementary_filter)

If you will be operating in a magnetically noisy environment, set `use_mag=0`. Magnetometer will still be read and published at `/imu/mag`, but will not be used for orientation measurements. Useful for indoor environments.

`kAngularVelocityThreshold`, `kAccelerationThreshold` and `kDeltaAngularVelocityThreshold` are thresholds for steady state detection. While the unit is with in these thresholds, it will calculate the gyro biases and accel gain. When the device is on steady mode, the green light should go on.

Adjust these values until you have the green light, when the sensor is stable.

`steady_state` adjust how many times the unit has to measure sensor values, within the thresholds, before going on steady state.

The `mag_offset` and `mag_soft_iron` parameters should be filled with data from your calibration software. Sensor board will auto correct for hard and soft iron errors.

See [CALIBRATION.md](CALIBRATION.md) for details.

#### GFSR

| GFSR | Description
| ---- | ----
| 0    | GFSR\_2000PS
| 1    | GFSR\_1000PS
| 2    | GFSR\_500PS
| 3    | GFSR\_250PS

Default is `GFSR_2000PS`

#### AFSR

| AFSR | Description
| ---- | ----
| 0    | AFSR\_2G	
| 1    | AFSR\_4G
| 2    | AFSR\_8G

Default is `AFSR_8G`

// TODO: the below statement needs to be corrected.
**For a ground based planar robot, use `GFSR_250PS` for gyro, and `AFSR_8G` for accelerometer.**

**Licenses**

|![tr000003](https://raw.githubusercontent.com/rosrider/fximu_doc/main/img/TR000003.png)   |![license](https://raw.githubusercontent.com/ROSRider/fximu_doc/main/img/license.png)|
|----|----|  
