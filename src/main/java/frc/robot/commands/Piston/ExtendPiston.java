// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Piston;

import frc.robot.subsystems.Pneumatics;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ExtendPiston extends InstantCommand {
  
  public ExtendPiston() {
    Pneumatics.m_doubleSolenoid.set(DoubleSolenoid.Value.kForward);
  }

}
