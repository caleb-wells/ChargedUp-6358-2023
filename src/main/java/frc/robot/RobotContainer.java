// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.Arm.ExtendArm;
import frc.robot.commands.Arm.RaiseArm;
import frc.robot.commands.Auto.MainAuto;
import frc.robot.commands.Piston.ExtendPiston;
import frc.robot.commands.Piston.RetractPiston;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Pneumatics;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

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
                MathUtil.applyDeadband(-m_driverController.getLeftY(), 0.10),
                MathUtil.applyDeadband(-m_driverController.getLeftX(), 0.10),
                MathUtil.applyDeadband(m_driverController.getRightX(), 0.10),
                false),
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

    new JoystickButton(m_copilotController, 5)
        .whileTrue(new RunCommand(
          () -> new ExtendArm(1)))
        .whileFalse(new RunCommand(
          () -> new ExtendArm(0)));
      
    new JoystickButton(m_copilotController, 6)
        .whileTrue(new RunCommand(
          () -> new ExtendArm(-1)))
        .whileFalse(new RunCommand(
          () -> new ExtendArm(0)));

    new JoystickButton(m_copilotController, 2)
        .whileTrue(new RunCommand(
            () -> new RaiseArm(0.5)))
        .whileFalse(new RunCommand(
            () -> new RaiseArm(0)));

    new JoystickButton(m_copilotController, 3)
        .whileTrue(new RunCommand(
            () -> new RaiseArm(-0.5)))
        .whileFalse(new RunCommand(
            () -> new RaiseArm(0)));

    new JoystickButton(m_copilotController, 4)
      .onTrue(new RunCommand(() -> new ExtendPiston(), m_piston));

    new JoystickButton(m_copilotController, 1)
      .onTrue(new RunCommand(() -> new RetractPiston(), m_piston));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new MainAuto();
  }
}
