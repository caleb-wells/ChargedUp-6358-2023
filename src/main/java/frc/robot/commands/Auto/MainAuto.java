// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drive.DriveForward;

public class MainAuto extends SequentialCommandGroup {

  public MainAuto() {
    addCommands(
      new DriveForward(1),
      //new WaitCommand(4),
      new DriveForward(-1));
  }

}
