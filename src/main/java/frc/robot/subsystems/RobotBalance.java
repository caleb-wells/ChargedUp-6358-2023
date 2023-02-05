// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import frc.robot.RobotContainer;

/** Add your docs here. */
public class RobotBalance {
    private static ADIS16470_IMU gyro = DriveSubsystem.m_gyro;
    private static DriveSubsystem robotDrive = RobotContainer.m_robotDrive;

    public static void balanceRobotonX() {
        gyro.setYawAxis(IMUAxis.kX);
        while(gyro.getAngle() < -5.0 || gyro.getAngle() > 5.0) {
            robotDrive.drive(
                0.1,
                0,
                0,
                false);
        }
    }

    public static void balanceRobotonY() {
        gyro.setYawAxis(IMUAxis.kY);
        while(gyro.getAngle() < -5.0 || gyro.getAngle() > 5.0) {
            robotDrive.drive(
                0,
                0.1,
                0,
                false);
        }
    }
}
