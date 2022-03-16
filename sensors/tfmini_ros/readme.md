# Laser scanner:

Basic implementation heavily borrowed from ros_tutorials by Tiziano Fiornzani
https://github.com/tizianofiorenzani/ros_tutorials/tree/master/laser_scanner_tfmini

Added ros launch wrapper and customized for servos in use.
Initial implementation used servoblaster library

Created new pigpio-blaster library due to the interaction of servoblaster and pgpio causing issues for the hall effect sensor.
This implementation has similar functionality compared to servoblaster and is designed for single servo

## Pinouts:

### Servo : Pi

- Red : 5V
- Brown : Gnd
- Orange : GPIO 18

### TFMini Plus: Pi

- Red : 5V
- Black : Gnd
- Green : RX
- White : Not connected

Current setup has serial port as ```/dev/ttyAMA0```

## Running Node:

- Source workspace
- run launch file :

``` roslaunch tfmini_ros tfmini_scanner.launch```

## Mounting on donkey car frame:

Used bracket from thingyverse, scaled 1.03 times for servo to fit.
https://www.thingyverse.com/thing:2401793

Currently sits inclined negative pitch, will use spacers to have positive pitch
Will redesign / modify design to be printed with correct pitch

# To do

- Verify angle max min and duty cycle -done
- Extend logic - done
- Figure out extrinsic transform from scanner to baselink -done