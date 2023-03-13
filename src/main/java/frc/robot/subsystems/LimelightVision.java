// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightVision extends SubsystemBase {
  /** Creates a new LimelightVision Subsystem. */
  
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");

  double targetOffsetAngle_Vertical = ty.getDouble(0.0);
  
  double angleToGoalDegrees = VisionConstants.limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    
  double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);

  //Calculate Distance
  double distanceFromLimelightToGoalInches = (VisionConstants.goalHeightInches - VisionConstants.limelightLensHeightInches)/Math.tan(angleToGoalRadians);

  public LimelightVision() { }

  @Override
  public void periodic() {
    //This method will be called once per scheduler run

    //Read values periodically
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);

    //Post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
  }
}