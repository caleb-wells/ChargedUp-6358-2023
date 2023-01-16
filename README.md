# Buhlean Operators Charged Up Repository v1.0

The Buhlean Operators code for the 2023 season *Charged Up*

Now featuring Swerve Drive!

Note that this is meant to be used with a drivetrain composed of four Swerve and Steer Modules, each configured with two SPARKS MAX, a NEO as the driving motor, a PG71 as the turning motor, and a REV Through Bore Encoder as the absolute turning encoder.

To get started, make sure you have calibrated the zero offsets for the absolute encoders in the `REV Hardware Client` using the `Absolute Encoder` tab under the associated turning SPARK MAX devices.

## Prerequisites
* SPARK MAX Firmware v1.6.1 - Adds features that are required for swerve
* REVLib v2023.1.1 - Includes APIs for the new firmware features
* WPILib v2023.1.1 - Newest version available at the time this repository was created. *Future versions may not be compatible with this project*

## Recommended Extensions
* Colorful Comments v1.1.0 or Higher
* Prettier Java v0.1.1 or Higher
* Window Colors v1.0.51 or Higher *By Stuart Robinson*
* General Java Extensions