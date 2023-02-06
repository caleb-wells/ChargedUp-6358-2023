// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
public class LEDController extends InstantCommand {
  public LEDController(double setting, Spark LED) {
    LED.set(setting);
    new WaitCommand(10); //This should make it wait 10 seconds...hopefully this works
    LED.set(Robot.defaultLEDColor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() { }
}
