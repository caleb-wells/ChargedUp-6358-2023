// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LEDs;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;

public class TimedSetLEDs extends SequentialCommandGroup {

  public TimedSetLEDs(double setting, Spark leds) {

    addCommands(
      new SetLEDs(setting, leds), 
      new WaitCommand(10), 
      new SetLEDs(Robot.defaultLEDColor, leds)
    );
  
  }
}
