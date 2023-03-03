// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Arm.ExtendArm;

public class MainAuto extends SequentialCommandGroup {
  
  private Timer timer = new Timer();
  public MainAuto() {
    timer.start();
    while (timer.get() <= 3) {
      new RunCommand(() -> new ExtendArm(1));
    }
    new RunCommand(() -> new ExtendArm(0));
    timer.stop();
    timer.reset();
   }
  
}
