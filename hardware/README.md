# Hardware notes

On the robot each axis had a potentiometer knob and a Raspberry Pi 5 ran
the control and the web page. This folder is about the part of that this
repo actually implements: reading the knobs.

## Knobs to the Pi

The Pi 5 has no analog inputs, so the pots go through two ADS1115 ADC
boards on I2C. `pot_teleop` expects:

| Knob | ADC address | Channel |
|---|---|---|
| base | 0x48 (ADDR to GND) | A0 |
| boom (actuator) | 0x48 | A1 |
| wrist pitch | 0x48 | A2 |
| wrist roll | 0x48 | A3 |
| gripper | 0x49 (ADDR to VDD) | A0 |

Each pot sits across 3.3 V and GND with the wiper on its channel. Both ADC
boards share the Pi's I2C bus:

    Pi pin 1  (3.3 V)   -> VDD on both boards, top of every pot
    Pi pin 3  (GPIO2)   -> SDA
    Pi pin 5  (GPIO3)   -> SCL
    Pi pin 6  (GND)     -> GND on both boards, bottom of every pot

Enable I2C (`sudo raspi-config`, Interface options), check the boards show
up with `i2cdetect -y 1`, install `python3-smbus2`, then

    ros2 run lathe_arm_teach pot_teleop --ros-args -p source:=ads1115

The ADC runs at gain 1 (+-4.096 V range), so 3.3 V reads about 26400
counts. `pots.ads1115_to_fraction` turns that into 0..1 and `pots.Channel`
handles pots that don't reach the ends or are wired backwards.

## Motors

The outputs (base motor, linear actuator, the three servos) are only
simulated here. In Gazebo ros2_control drives the joints. On real hardware
a ros2_control hardware interface for the motor drivers would replace
`gz_ros2_control/GazeboSimSystem` in the URDF, and nothing above it changes.

## Resolution

`python3 analysis/pot_resolution.py` works out how fine each knob is. Short
version: the ADC isn't the limit, the deadband in the filter is (about 1.4
degrees on the base and 1.5 mm of actuator stroke), and that deadband is
what keeps the arm from twitching when nobody touches the knobs.
