// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import frc.robot.subsystems.ArmSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RetractArm extends CommandBase {
  public static ArmSubsystem arm = new ArmSubsystem();

  public RetractArm(double speed) {
    if(!arm.isFullyExtendedIn()) {
      arm.setMotorSpeed(speed);
    }
  }

}