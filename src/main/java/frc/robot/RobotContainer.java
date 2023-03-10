// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.Arm.MoveArm;
import frc.robot.commands.Auto.MainAuto;
import frc.robot.commands.Piston.ExtendPiston;
import frc.robot.commands.Piston.RetractPiston;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Pneumatics;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.HashMap;
import java.util.List;
import java.util.*;
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
  public static final DriveSubsystem m_robotDrive = new DriveSubsystem();

  public final XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  public final Joystick m_copilotController = new Joystick(OIConstants.kCoPilotControllerPort);

  public static Spark m_leds = new Spark(0);

  public final Pneumatics m_piston = new Pneumatics();

  // Beginning of PathPlanner Code
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
  // End of PathPlanner Code

  Command fullAuto = autoBuilder.fullAuto(pathGroup);

  Command mainAuto = new MainAuto();
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    configureButtonBindings();

    // Configure default commands --> This is how the robot drives, should not need to be adjusted, if the robot is driving
    // improperly, it is very likely that it is somewhere else in the code *cough* SwerveModule.java *cough*
    m_robotDrive.setDefaultCommand(
        new RunCommand(
            () -> m_robotDrive.drive(
                MathUtil.applyDeadband(m_driverController.getLeftY(), 0.23),
                MathUtil.applyDeadband(-m_driverController.getLeftX(), 0.23),
                MathUtil.applyDeadband(m_driverController.getRightX(), 0.23),
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

    new JoystickButton(m_copilotController, 1)
        .onTrue(new RunCommand(() -> new ExtendPiston(), m_piston))
        .onFalse(new RunCommand(() -> new RetractPiston(), m_piston));

    new JoystickButton(m_copilotController, 2)
        .whileTrue(new RunCommand(
            () -> new MoveArm(0.5)))
        .whileFalse(new RunCommand(
            () -> new MoveArm(0)));

    new JoystickButton(m_copilotController, 3)
        .whileTrue(new RunCommand(
            () -> new MoveArm(-0.5)))
        .onFalse(new RunCommand(
            () -> new MoveArm(0)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return mainAuto;
  }
}
