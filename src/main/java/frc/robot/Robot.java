// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Auto.BalanceAuto;
import frc.robot.commands.Auto.MainAuto;
import frc.robot.commands.Auto.MobilityAuto;
import frc.robot.commands.Drive.SetCoastModeCommand;
import frc.robot.commands.LEDs.SetLEDs;
import frc.robot.commands.Piston.ExtendPiston;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.SwerveModule;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private static final String m_mainAuto = "Main Auto";

  private static final String m_balanceAuto = "Balancing Auto";

  private static final String m_mobilityAuto = "Mobility Auto";

  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  public RobotContainer m_robotContainer;

  public static SwerveModule m_SDModule = DriveSubsystem.m_frontRight;

  public static Spark m_leds = RobotContainer.m_leds;

  public static String kAllianceString = DriverStation.getAlliance().toString();

  public static double defaultLEDColor = (kAllianceString.contains("Blue") ? 0.85 : 0.61); //Sets the color to our alliance color

  public static InstantCommand setLEDDefault = new InstantCommand(() -> new SetLEDs(defaultLEDColor, m_leds));

  public static ArmSubsystem armSubsystem = RobotContainer.m_armSubsystem;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings
    m_robotContainer = new RobotContainer();

    //Sets the color of the LEDs on the robot
    kAllianceString = DriverStation.getAlliance().toString();

    if(kAllianceString.contains("Red")) {
      defaultLEDColor = 0.61;
    }
    if(kAllianceString.contains("Blue")) {
      defaultLEDColor = 0.85;
    }

    m_chooser.setDefaultOption("Main Auto", m_mainAuto);

    m_chooser.addOption("Balance Auto", m_balanceAuto);

    m_chooser.addOption("Mobility Auto", m_mobilityAuto);

    SmartDashboard.putData("Auto choices", m_chooser);
    
    new Trigger(this::isEnabled)
      .negate()
      .whileTrue(new SetCoastModeCommand(RobotContainer.m_robotDrive));
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    //Sets the color of the LEDs on the robot
    kAllianceString = DriverStation.getAlliance().toString();
    if(kAllianceString.contains("Red")) {
      defaultLEDColor = 0.61;
    }
    if(kAllianceString.contains("Blue")) {
      defaultLEDColor = 0.85;
    }

    /*
    double turnEncoder = m_SDModule.getTurningEncoderValue();
    double driveEncoder = m_SDModule.getDrivingEncoderValue();
    double gyroAngle = m_gyro.getAngle();
    double encoderConversionValue = m_SDModule.getPositionConversionFactor();
    String position = m_SDModule.getPosition().toString();
    String state = m_SDModule.getState().toString();
    SmartDashboard.putNumber("Front Right Turn Encoder", turnEncoder);
    SmartDashboard.putNumber("Front Right Drive Encoder", driveEncoder);
    SmartDashboard.putNumber("Front Right Drive Encoder Position Conversion", encoderConversionValue);
    SmartDashboard.putString("Front Right Position", position);
    SmartDashboard.putString("Front Right State", state);
    SmartDashboard.putNumber("Gyro", gyroAngle);
    */
  }

  /* This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() { }

  @Override
  public void disabledPeriodic() { }

  /* This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    //This will be useful in the future when we have several autonomous commands
    String m_autoSelected = SmartDashboard.getString("Auto Selector", "Default"); 
    m_autoSelected = m_chooser.getSelected();

    switch (m_autoSelected) {
      case m_balanceAuto:
        m_autonomousCommand = new BalanceAuto();
        break;
      case m_mainAuto:
        m_autonomousCommand = new MainAuto();
        break;
      case m_mobilityAuto:
        m_autonomousCommand = new MobilityAuto();
        break;
      default:
        m_autonomousCommand = new MainAuto();
        break;
    }

    System.out.println("Auto selected: " + m_autoSelected);

    //Schedules the selected autonomous command from the switch statement above
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    RobotContainer.m_robotDrive.setBrakeMode(false);
  }

  /* This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() { }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    RobotContainer.m_robotDrive.setBrakeMode(false);

    //Sets the color of the LEDs on the robot
    kAllianceString = DriverStation.getAlliance().toString();
    if(kAllianceString.contains("Red")) {
      defaultLEDColor = 0.61;
    }
    if(kAllianceString.contains("Blue")) {
      defaultLEDColor = 0.85;
    }

    m_leds.set(defaultLEDColor);
  }

  /* This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() { 
    System.out.println(armSubsystem.getDistance()/360);
    
    if(Timer.getMatchTime() >= 149) {
      System.out.println("Time is almost out! Exending the Piston...");
      new ExtendPiston();
    }
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    
    RobotContainer.m_robotDrive.setBrakeMode(false);
  }

  /* This function is called periodically during test mode. */
  @Override
  public void testPeriodic() { }
}
