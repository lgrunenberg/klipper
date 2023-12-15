# Servo42C

This document provides information on using Makerbase Servo42C Closed-Loop
Servo Controllers (and clones) in UART Mode.

Klipper can also use Servo42C servos in their step/direction mode.
However, when the servos are in this mode, no special Klipper
configuration is needed and the advanced Klipper features discussed in
this document are not available.

## Tuning motor current

A higher driver current increases positional accuracy and torque.
However, a higher current also increases the heat produced by the
stepper motor and the stepper motor driver. If the stepper motor
driver gets too hot it might be destroyed (If it gets very hot it may also 
melt plastic parts attached to it or near it.)

As a general tuning tip, prefer higher current values as long as the
stepper motor and the stepper motor driver does not get too hot. In
general, it is okay for the stepper motor to feel warm, but it should not
become so hot that it is painful to touch.

## Sensorless Homing

Sensorless homing allows to home an axis without the need for a
physical limit switch. Instead, the carriage on the axis is moved into
the mechanical limit making the stepper motor lose steps. The stepper
driver senses the lost steps and indicates this to the controlling MCU
(Klipper) by a value in a specific register. This information can be used by 
Klipper as end stop for the axis.

This guide covers the setup of sensorless homing for the X axis of
your (cartesian) printer. However, it works the same with all other
axes (that require an end stop). You should configure and tune it for
one axis at a time.

### Limitations

Be sure that your mechanical components are able to handle the load of
the carriage bumping into the limit of the axis repeatedly. Especially
leadscrews might generate a lot of force. Homing a Z axis by bumping
the nozzle into the printing surface might not be a good idea. For
best results, verify that the axis carriage will make a firm contact
with the axis limit.

Further, sensorless homing might not be accurate enough for your
printer. While homing X and Y axes on a cartesian machine can work
well, homing the Z axis is generally not accurate enough and may
result in an inconsistent first layer height. Homing a delta printer
sensorless is not advisable due to missing accuracy.

### Prerequisites

TODO

### Tuning

TODO

### Tips for sensorless homing on CoreXY

It is possible to use sensorless homing on the X and Y carriages of a
CoreXY printer. Klipper uses the `[stepper_x]` stepper to detect
stalls when homing the X carriage and uses the `[stepper_y]` stepper
to detect stalls when homing the Y carriage.

Use the tuning guide described above to find the appropriate "stall
sensitivity" for each carriage, but be aware of the following
restrictions:
1. When using sensorless homing on CoreXY, make sure there is no
   `hold_current` configured for either stepper.
2. While tuning, make sure both the X and Y carriages are near the
   center of their rails before each home attempt.
3. After tuning is complete, when homing both X and Y, use macros to
   ensure that one axis is homed first, then move that carriage away
   from the axis limit, pause for at least 2 seconds, and then start
   the homing of the other carriage. The move away from the axis
   avoids homing one axis while the other is pressed against the axis
   limit (which may skew the stall detection). The pause is necessary
   to ensure the driver's stall flag is cleared prior to homing again.

An example CoreXY homing macro might look like:
```
[gcode_macro HOME]
gcode:
    G90
    # Home Z
    G28 Z0
    G1 Z10 F1200
    # Home Y
    G28 Y0
    G1 Y5 F1200
    # Home X
    G4 P2000
    G28 X0
    G1 X5 F1200
```
