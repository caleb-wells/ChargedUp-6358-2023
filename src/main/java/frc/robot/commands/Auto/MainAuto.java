// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.Arm.ExtendArm;

public class MainAuto extends SequentialCommandGroup {
  private RobotContainer m_container;
  private Timer timer = new Timer();
  public Command fullAuto = m_container.autoBuilder.fullAuto(m_container.pathGroup);
  public MainAuto() {
    //Start Timer
    timer.start();

    //While auto has been running for less than 3 seconds extend the arm
    while (timer.get() <= 3) {
      new RunCommand(() -> new ExtendArm(1));
    }
    //Once those three seconds have elapsed stop running arm
    new RunCommand(() -> new ExtendArm(0));
    //Drive robot forward
    fullAuto.schedule();
    //Hopefully fix swerve error
    m_container.m_robotDrive.drive(0, 0, 0, false);
    
    //Stop and Reset Timer
    timer.stop();
    timer.reset();
   }
  
}
