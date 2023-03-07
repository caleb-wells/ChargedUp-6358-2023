// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  private final Encoder m_encoder;
  private double distance = 0;

  public ArmSubsystem() {
    m_encoder = new Encoder(ArmConstants.armEncoderChannel1, ArmConstants.armEncoderChannel2, false, CounterBase.EncodingType.k4X);
    m_encoder.setDistancePerPulse(1);
  }

  public boolean isFullyExtended() {

    distance = (m_encoder.getDistance()/360);

    if(Math.abs(distance) <= 13) {
      return true;
    } else {
      return false;
    }
  }

  public double getDistance() {
    return m_encoder.getDistance();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
