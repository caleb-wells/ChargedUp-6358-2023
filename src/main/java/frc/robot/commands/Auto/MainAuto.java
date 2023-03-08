// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import frc.robot.RobotContainer;
import frc.robot.commands.Arm.ExtendArm;
import frc.robot.subsystems.DriveSubsystem;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


public class MainAuto extends SequentialCommandGroup {
  private Timer timer = new Timer();
  private DriveSubsystem m_drive = RobotContainer.m_robotDrive;
  
  // Beginning of PathPlanner Code
  // This will load the file "MainAuto.path" and generate it with a max velocity of 2 m/s and a max acceleration of 0.5 m/s^2
  // for every path in the group
  public List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("MainAuto", new PathConstraints(2, 0.5));

  private Map<String, Command> eventMap = new HashMap<>();

  // Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
  public SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
    m_drive::getPose, // Pose2d supplier
    m_drive::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
    m_drive.getKinematics(), // SwerveDriveKinematics
    new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
    new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
    m_drive::setModuleStates, // Module states consumer used to output to the drive subsystem
    eventMap,
    true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
    m_drive // The drive subsystem. Used to properly set the requirements of path following commands
   );
  // End of PathPlanner Code

  public Command fullAuto = autoBuilder.fullAuto(pathGroup);

  public MainAuto() {
    //Start Timer
    timer.start();

    //While auto has been running for less than 3 seconds extend the arm
    while (timer.get() <= 3) {
      new RunCommand(() -> new ExtendArm(1));
    }

    //Once those three seconds have elapsed stop running arm
    new RunCommand(() -> new ExtendArm(0));

    //Schedule Auto command so that the robot moves (In Theory)
    fullAuto.schedule();
    
    //Hopefully fix swerve error
    m_drive.drive(0, 0, 0, false);
    
    //Stop and Reset Timer
    timer.stop();

    timer.reset();
   }
  
}
