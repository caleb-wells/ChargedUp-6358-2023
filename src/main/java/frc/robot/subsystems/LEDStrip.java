// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;

/** Add your docs here. */
public class LEDStrip {
    //Creates the LEDs
    public static Spark m_underglowLEDs = new Spark(0);
    
    public static void set(double value) {
        m_underglowLEDs.set(value);
        new WaitCommand(10); //This should make it wait 10 seconds...hopefully this works
        m_underglowLEDs.set(Robot.defaultLEDColor);
    }
}
