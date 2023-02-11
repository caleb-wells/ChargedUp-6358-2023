// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Motors extends SubsystemBase {
  //* Creates a new Motors. */

  public Spark armMotor = new Spark(Constants.ExtraMotorConstants.armMotor);

  public Motors() { }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runMotor(Spark motor, double speed){
    motor.set(speed);
  }
  }

