// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

/** Add your docs here. */
public class LEDStrip {
    //Creates the LEDs
    public static Spark m_underglowLEDs = new Spark(0);

    public static Spark getLEDs() {
        return m_underglowLEDs;
    } 
}
