// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LEDStrip;

//~ Instant commands call initialisze and then end immediately.
//! They don't need any other life-cycle methods.
public class SetCoastModeCommand extends InstantCommand {
  private DriveSubsystem m_subsystem = RobotContainer.m_robotDrive;
  private Spark m_leds = LEDStrip.m_underglowLEDs;
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

  //! Allow this command to run when disabled DO NOT REMOVE
  @Override
  public boolean runsWhenDisabled() {
      return true;
  }
}
