// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.RobotBalance;
import frc.robot.commands.LEDController;
import frc.robot.commands.RetractPiston;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LEDStrip;
import frc.robot.subsystems.Piston;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
//import frc.robot.subsystems.Motors; //& Uncomment when it is time to use additional motors

//?New Auto Imports
//!Java Auto Imports
import java.util.HashMap;
import java.util.List;
import java.util.*;
//!PathPlanner Imports
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  //^ The Robot's Subsystems
  public final static DriveSubsystem m_robotDrive = new DriveSubsystem();
  //? Will uncomment once it is required, currently the SPARK is off of the robot
  //private final Motors m_motors = new Motors(); <-- Will uncomment once it is time for other motors.

  //^ The Driver's Controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  //^ The CoPilots Controller
  Joystick m_copilotController = new Joystick(OIConstants.kCoPilotControllerPort);

  //^LED Instances
  private Spark m_LEDs = LEDStrip.get();

  public Piston m_piston = new Piston();

  //*Beginning of PathPlanner Code
  // This will load the file "MainAuto.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
  // for every path in the group
  public static List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("MainAuto", new PathConstraints(4, 3));

  private Map<String, Command> eventMap = new HashMap<>();

  // Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
  SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
    m_robotDrive::getPose, // Pose2d supplier
    m_robotDrive::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
    m_robotDrive.getKinematics(), // SwerveDriveKinematics
    new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
    new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
    m_robotDrive::setModuleStates, // Module states consumer used to output to the drive subsystem
    eventMap,
    true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
    m_robotDrive // The drive subsystem. Used to properly set the requirements of path following commands
   );
  //*End of PathPlanner Code

  /**
   * *The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    //^ Configure the button bindings
    configureButtonBindings();

    // Configure default commands --> This is how the robot drives, should not need to be adjusted, if the robot is driving
    // improperly, there is a greater likelyhood that it is somewhere else in the code *cough* SwerveModule.java *cough*
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        //! Keep Robot Centric until debugging has finished
        new RunCommand(
            () -> m_robotDrive.drive(
                MathUtil.applyDeadband(-m_driverController.getLeftY(), 0.10),
                MathUtil.applyDeadband(-m_driverController.getLeftX(), 0.10),
                MathUtil.applyDeadband(-m_driverController.getRightX(), 0.10),
                true),
            m_robotDrive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {

    new JoystickButton(m_driverController, 1)
        .onTrue(new RunCommand(() -> new RetractPiston(), m_piston));
    
    //Set LEDs to Purple
    new JoystickButton(m_driverController, 2)
        .whileTrue(new RunCommand(
            () -> new LEDController(0.91, m_LEDs),
                m_robotDrive))
        .whileFalse(new RunCommand(
            () -> new LEDController(Robot.defaultLEDColor, m_LEDs),
                m_robotDrive));

    //Set LEDs to Yellow
    new JoystickButton(m_driverController, 3)
        .whileTrue(new RunCommand(
            () -> new LEDController(0.69, m_LEDs),
                m_robotDrive))
        .whileFalse(new RunCommand(
            () -> new LEDController(Robot.defaultLEDColor, m_LEDs),
                m_robotDrive));;

    //Calls setX() method in DriveSubsystem
    new JoystickButton(m_driverController, 4)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));
    
    //Balance Robot on Charging Station on X axis
    new JoystickButton(m_driverController, 5)
        .whileTrue(new RunCommand(() -> RobotBalance.balanceRobotOnX(), m_robotDrive));

    //Balance Robot on Charging Station on Y axis
    new JoystickButton(m_driverController, 6)
        .whileTrue(new RunCommand(() -> RobotBalance.balanceRobotOnY(), m_robotDrive));
    
    /*new JoystickButton(m_driverController, 3)
        .whileTrue(new RunCommand(
            () -> m_motors.runMotor(m_motors.armMotor, .75),
             m_motors));*/
    
    /*new JoystickButton(m_driverController, 3)
        .whileFalse(new RunCommand(
            () -> m_motors.runMotor(m_motors.armMotor, 0),
            m_motors));*/

    /*new JoystickButton(m_driverController, 4)
        .whileTrue(new RunCommand(
            () -> m_motors.runMotor(m_motors.armMotor, -0.75),
            m_motors));*/
        
    /*new JoystickButton(m_driverController, 4)
        .whileFalse(new RunCommand(
            () -> m_motors.runMotor(m_motors.armMotor, 0),
            m_motors));*/
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    //Creates the main auto command, this will be set as m_autonomousCommand in Robot.java
    Command MainAuto = autoBuilder.fullAuto(pathGroup);

    return MainAuto;
  }
}