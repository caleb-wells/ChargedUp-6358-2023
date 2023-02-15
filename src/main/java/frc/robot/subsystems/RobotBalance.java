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

    public static void balanceRobotOnX() {
        gyro.setYawAxis(IMUAxis.kX);
        double xSpeed = 0;
        while(gyro.getAngle() < -4.0 || gyro.getAngle() > 4.0) {
            xSpeed = (gyro.getAngle()/5)/3;
            robotDrive.drive(
                xSpeed,
                0,
                0,
                false);
        }
        gyro.setYawAxis(IMUAxis.kZ);
    }

    public static void balanceRobotOnY() {
        gyro.setYawAxis(IMUAxis.kY);
        double ySpeed = 0;
        while(gyro.getAngle() < -4.0 || gyro.getAngle() > 4.0) {
            ySpeed = (gyro.getAngle()/5)/3;
            robotDrive.drive(
                0,
                ySpeed,
                0,
                false);
        }
        gyro.setYawAxis(IMUAxis.kZ);
    }
}
