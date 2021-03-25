**This repository contains the fximu package. This is the only package you need to install in order to run fximu.**

### Introduction

FXIMU utilizes a port of ROS's complementary filter on the TM4C123 MCU using the NXP semiconductor FXOS8700 accelerometer magnetometer and FXAS21002 gyro sensor.

![fximu v1c](https://raw.githubusercontent.com/rosrider/fximu_doc/main/img/fximu_v1c.jpg)

#### Documentation Contents

See [QUICKSTART](QUICK.md) for instructions on how to start the device.

See [PARAMETERS](PARAMS.md) for description of parameters.

See [CALIBRATION](CALIBRATION.md) for instructions on calibration. The device comes calibrated, but once you mount it to your application, you might need to recalibrate it.

See [HOWTO](HOWTO.md) on tips on operation.

The firmware code is available at https://github.com/altineller/fximu_firmware

The hardware files are available at https://github.com/altineller/fximu_hardware

#### Features

**It works with ROS without any problems, does not require drivers, uses standard message types, and is open source.**

Sensor data is directly fed to the complementary filter, updated up to 400hz. The filter then queried to get quaternion data at a desired output rate by using a output\_rate\_divider parameter.

**Hardware is configurable using a fximu_params.yaml**

Here is example parameters file:

```
params/imu: {
calibration_mode: 0,
sensor_read_rate: 400, output_rate_divider: 8,
adaptive_gain: 1, bias_estimation: 1,
gain_acc: 0.02, gain_mag: 0.01, bias_alpha: 0.1,
imu_frame_id: "base_imu_link", mag_frame_id: "mag_imu_link",
gfsr: 2, afsr: 1,
steady_limit: 32,
world_frame: 0,
use_mag: 1,
kAngularVelocityThreshold: 0.06, kAccelerationThreshold: 0.35, kDeltaAngularVelocityThreshold: 0.05,
mag_offset_x: 27.00, mag_offset_y: 78.14, mag_offset_z: 85.36,
mag_soft_iron_ix: 0.987, mag_soft_iron_iy: -0.040, mag_soft_iron_iz: -0.021,
mag_soft_iron_jx: -0.040, mag_soft_iron_jy: 1.006, mag_soft_iron_jz: 0.006,
mag_soft_iron_kx: -0.021, mag_soft_iron_ky: 0.006, mag_soft_iron_kz: 1.009
}
```

See [Parameters](PARAMS.md) for detailed explanation of each parameter.

The device is equipped with a red and green led. Opon reset, the device will blink green first, then red.

If loading one of the parameters fails for `calibration_mode=2` or `calibration_mode=3`, FXIMU will be disabled, and the red led will blink long.

During normal operation, if ROS connection is active, red led will turn on faintly, depending on your loop speed.

If the device is at steady state, then green led will turn on faintly, indicating that the device is on stationary state.


#### Credits

_Roberto G. Valenti_ for writing the original complementary filter.

_Vitor Matos_ for writing the rosserial tivac library.

_Charles Tsai_ and user _cb1_mobile_ from e2e.ti.com forums, for help on debugging the problems on sensor chipset.

_Melih Karakelle_ for advice on the development of the circuit.

#### Licenses

|![tr000003](https://raw.githubusercontent.com/rosrider/fximu_doc/main/img/TR000003.png)   |![license](https://raw.githubusercontent.com/ROSRider/fximu_doc/main/img/license.png)|
|----|----|




