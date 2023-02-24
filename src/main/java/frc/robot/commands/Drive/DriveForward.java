// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveForward extends CommandBase {
  /** Creates a new DriveForward. */
  public DriveForward(double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    DriveSubsystem.driveForward(speed);
  }
  
}
