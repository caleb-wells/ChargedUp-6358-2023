// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static).
 *
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    //& Driving Parameters - Note that these are not the maximum capable speeds of
    //& the robot, rather, the allowed maximum speeds

    public static final double kMaxSpeedMetersPerSecond = 3.4;

    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    //Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(11.5);

    //Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(21);

    //Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    //Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    //&Driving SPARK MAX IDs
    //Red
    public static final int kFrontLeftDrivingCanId = 9;
    //Blue
    public static final int kRearLeftDrivingCanId = 6;
    //Orange
    public static final int kFrontRightDrivingCanId = 8;
    //Yellow
    public static final int kRearRightDrivingCanId = 7;

    //&Turning SPARK MAX IDs
    //Red
    public static final int kFrontLeftTurningCanId = 2;
    //Blue
    public static final int kRearLeftTurningCanId = 4;
    //Orange
    public static final int kFrontRightTurningCanId = 5;
    //Yellow
    public static final int kRearRightTurningCanId = 3;
 
    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {

    //& NEOS
    //Pinion teeth for drive motor - 12 tooth spur gear is driven by CIM Falcon 500 Motor - Per AndyMark
    public static final int kDrivingMotorPinionTeeth = 12;

    //~Taken from SwerveControllerCommand project and are used to create a new ProfiledPIDController
    public static final double kMaxModuleAngularSpeedRadiansPerSecond = 2 * Math.PI;
    public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 2 * Math.PI;
    public static final double kPModuleTurningController = 1;

    //Invert the turning encoder, since the output shaft rotates in the opposite direction of
    //the steering motor in the Swerve Module.
    public static final boolean kTurningEncoderInverted = false;

    //Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.1;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    
    //45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (40.0 * 20) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction;

    //Driving Encoder Config
    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction) / 60.0; // meters per second

    //Turning Encoder Config
    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    //PID Mins and Maxs
    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.007;
    public static final double kDrivingI = 0.001;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    //Current limits <- Prevents Brownouts and high battery draw
    public static final int kDrivingMotorCurrentLimit = 55; // amps
    public static final int kTurningMotorCurrentLimit = 25; // amps 
  }

  public static final class OIConstants {
    //These are the constants for Operator Input, this is where we tell RobotContainer what ports to use for Input for both the Pilot(Driver) and CoPilot
    public static final int kDriverControllerPort = 0;
    public static final int kCoPilotControllerPort = 1;
  }

  //Constants for autonomous mode, this sets the speed and controllers for autonomous, could be very important in the future
  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 1;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  //Defines NEO Motor Constants, should NEVER be changed
  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class ExtraMotorConstants {
    public static final int elevatorMotor = 1;
    public static final int armMotor = 2;
  }
}
