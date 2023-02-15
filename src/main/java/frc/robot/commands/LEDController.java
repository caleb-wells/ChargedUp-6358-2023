// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

//import edu.wpi.first.wpilibj.Timer;
//import frc.robot.Robot;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class LEDController extends InstantCommand {

  public LEDController(double setting, Spark LED) {
    LED.set(setting);
  }

  //This code creates a new thread, however this does not solve our issue with the LEDs seen as adding the thread will freeze everything
  /*new Thread(() -> {
        try {
          LED.set(setting);
          System.out.println("Timer is about to be called");
          Timer.delay(5000);
          System.out.println("Timer finished successfully, thread sleep is being called now");
          Thread.sleep(5000);
          System.out.println("Thread sleep finished successfully!");
          LED.set(Robot.defaultLEDColor);
        } catch (Exception e) { }
      }).start();*/

  // Called when the command is initially scheduled.
  @Override
  public void initialize() { }
}
