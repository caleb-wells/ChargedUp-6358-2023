// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

public class SetCoastModeCommand extends InstantCommand {
  private DriveSubsystem m_subsystem = RobotContainer.m_robotDrive;

  /**
   * Sets the Coast Mode
   * @param subsystem
   */
  public SetCoastModeCommand(DriveSubsystem subsystem) {
      m_subsystem = subsystem;
      addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    m_subsystem.setBrakeMode(false); // Disable brake mode
  }

  /*@Override
  public boolean runsWhenDisabled() {
      return true;
  }*/
}
