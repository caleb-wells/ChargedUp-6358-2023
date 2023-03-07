// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Balance;

import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj2.command.CommandBase;
//import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;

public class BalanceX extends CommandBase {
  private static ADIS16470_IMU gyro = DriveSubsystem.m_gyro;
  private static DriveSubsystem robotDrive = RobotContainer.m_robotDrive;

  public BalanceX() {
    //gyro.setYawAxis(IMUAxis.kX);
    double xSpeed = 0;

    while(gyro.getAngle() < -4.0 || gyro.getAngle() > 4.0) {
        xSpeed = (gyro.getAngle()/5)/3;
        robotDrive.drive(xSpeed, 0, 0, false);
    }
    //gyro.setYawAxis(IMUAxis.kZ);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
