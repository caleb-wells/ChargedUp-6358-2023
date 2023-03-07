// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

public class ExtendArm extends CommandBase {
  public static final VictorSP armMotor = new VictorSP(Constants.ExtraMotorConstants.armMotor);
  public static ArmSubsystem arm = new ArmSubsystem();

  public ExtendArm(double speed) {
    if(!arm.isFullyExtended()) {
      armMotor.set(speed); 
    }
  }

}
