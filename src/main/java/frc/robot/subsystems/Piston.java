// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Piston extends SubsystemBase {
  /** Creates a new Piston. */
  private static final int PH_CAN_ID = 10;
  private static int forwardChannel = 8;
  private static int reverseChannel = 9;
  public static PneumaticHub m_pH = new PneumaticHub(PH_CAN_ID);
  public static DoubleSolenoid m_doubleSolenoid = m_pH.makeDoubleSolenoid(forwardChannel,reverseChannel);

  public Piston() { }

  public DoubleSolenoid getSolenoid() {
    return m_doubleSolenoid;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
