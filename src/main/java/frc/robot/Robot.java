// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//~ Other Imports
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.LEDController;
import frc.robot.commands.SetCoastModeCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LEDStrip;
import frc.robot.subsystems.SwerveModule;
import edu.wpi.first.wpilibj.ADIS16470_IMU;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private ADIS16470_IMU m_gyro = DriveSubsystem.m_gyro;

  public static SwerveModule m_testModule = DriveSubsystem.m_frontRight;

  private static Spark m_leds = LEDStrip.m_underglowLEDs;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    m_leds.set(0.63);
    //! Turn brake mode off shortly after the robot is disabled
    new Trigger(this::isEnabled)
      .negate()
      .debounce(6) //Should be greater than 5 seconds
      .whileTrue(new SetCoastModeCommand(RobotContainer.m_robotDrive));
    new Trigger(this::isEnabled)
      .negate()
      .whileTrue(new LEDController(0.73, m_leds));
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
    //*Vars
    double encoder = m_testModule.getTurningEncoderValue();
    String position = m_testModule.getPosition().toString();
    String state = m_testModule.getState().toString();
    double gyroAngle = m_gyro.getAngle();
    //*SmartDashboard Keys
    SmartDashboard.putNumber("Front Right Encoder", encoder);
    SmartDashboard.putString("Front Right Position", position);
    SmartDashboard.putString("Front Right State", state);
    SmartDashboard.putNumber("Gyro", gyroAngle);
  }

  /* This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() { }

  @Override
  public void disabledPeriodic() { }

  /* This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    /**
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    RobotContainer.m_robotDrive.setBrakeMode(true); //? Enable Brake Mode
  }

  /* This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() { }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    RobotContainer.m_robotDrive.setBrakeMode(true); //? Enable Brake Mode
  }

  /* This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() { }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    RobotContainer.m_robotDrive.setBrakeMode(true); //? Enable Brake Mode
  }

  /* This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
