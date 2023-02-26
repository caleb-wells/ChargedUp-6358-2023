// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

public class RaiseArm extends CommandBase {
  
  public static VictorSP elevatorMotor = new VictorSP(Constants.ExtraMotorConstants.elevatorMotor);

  public RaiseArm(double speed) {
    elevatorMotor.set(speed);
  }

}
