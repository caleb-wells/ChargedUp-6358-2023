// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

public class MobilityAuto extends CommandBase {
  /** Creates a new MobilityAuto. */
  private DriveSubsystem m_drive = RobotContainer.m_robotDrive;
  
  // Beginning of PathPlanner Code
  // This will load the file "Mobility.path" and generate it with a max velocity of 2 m/s and a max acceleration of 1.9 m/s^2
  // for every path in the group
  public List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("Mobility", new PathConstraints(2, 1.9));
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

  public MobilityAuto() {
    //Schedule Auto command so that the robot moves (In Theory)
    fullAuto.schedule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
