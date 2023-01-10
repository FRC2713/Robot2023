// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
// liam sais hi :)
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.SwerveIO.module.ModuleInfo;
import frc.robot.subsystems.SwerveIO.module.SwerveModules;
import frc.robot.util.PIDFFGains;
import lombok.experimental.UtilityClass;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  private Constants() {
    throw new AssertionError();
  }

  public static final boolean tuningMode = false;
  public static final int zero = 0; // in case you need a zero :)

  public static final class RobotMap {
    private RobotMap() {
      throw new AssertionError();
    }

    public static final int pigeonCANId = 20;

    // MOTORS
    // FrontLeft
    public static final int frontLeftDrive = 1;
    public static final int frontLeftAzi = 8;
    public static final int frontLeftAzimuthEncoder = 0;
    public static final double frontLeftOffset = 0.1124;

    // FrontRight
    public static final int frontRightDrive = 3;
    public static final int frontRightAzi = 2;
    public static final int frontRightAzimuthEncoder = 1;
    public static final double frontRightOffset = 0.6028;

    // BackLeft
    public static final int backLeftDrive = 4;
    public static final int backLeftAzi = 5;
    public static final int backLeftAzimuthEncoder = 2;
    public static final double backLeftOffset = 0.0626;

    // BackRight
    public static final int backRightDrive = 6;
    public static final int backRightAzi = 7;
    public static final int backRightAzimuthEncoder = 3;
    public static final double backRightOffset = 0.775;
  }

  public static final class DriveConstants {
    private DriveConstants() {
      throw new AssertionError();
    }

    public static final double kJoystickTurnDeadzone = 0.04;
    public static final double wheelDiameter = 4;
    public static final double gearRatio = 6.12;
    public static final double distPerPulse =
        (1.0 / gearRatio) * Units.inchesToMeters(wheelDiameter) * Math.PI;

    public static final double maxSwerveVel = Units.feetToMeters(16.0 * 0.75);
    public static final double maxSwerveAzi = Math.PI;
    public static final double maxSwerveAccel = Units.feetToMeters(0.5);
    public static final double maxRotationalSpeedRadPerSec = Units.degreesToRadians(180);

    public static final int currentLimit = 65;

    public static final double kModuleDistanceFromCenter = Units.inchesToMeters(12.375);

    private static final Translation2d frontLeftLocation =
        new Translation2d(
            DriveConstants.kModuleDistanceFromCenter, DriveConstants.kModuleDistanceFromCenter);
    private static final Translation2d frontRightLocation =
        new Translation2d(
            DriveConstants.kModuleDistanceFromCenter, -DriveConstants.kModuleDistanceFromCenter);
    private static final Translation2d backLeftLocation =
        new Translation2d(
            -DriveConstants.kModuleDistanceFromCenter, DriveConstants.kModuleDistanceFromCenter);
    private static final Translation2d backRightLocation =
        new Translation2d(
            -DriveConstants.kModuleDistanceFromCenter, -DriveConstants.kModuleDistanceFromCenter);

    public static final SwerveDriveKinematics kinematics =
        new SwerveDriveKinematics(
            frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

    private static final double bumperlessRobotLength = Units.inchesToMeters(30);
    private static final double bumperlessRobotWidth = Units.inchesToMeters(30);
    private static final double bumperThickness = Units.inchesToMeters(3);

    public static final double fullRobotWidth = bumperlessRobotWidth + bumperThickness * 2;
    public static final double fullRobotLength = bumperlessRobotLength + bumperThickness * 2;

    public static final PIDFFGains kHeadingControllerGains =
        PIDFFGains.builder("Heading Controller").kP(5).kD(0.001000).tolerance(0).build();
    public static final double headingControllerDriverChangeRate = 4;

    public static final ModuleInfo frontLeft =
        ModuleInfo.builder()
            .name(SwerveModules.FRONT_LEFT)
            .driveGains(Constants.DriveConstants.Gains.kDefaultDrivingGains)
            .azimuthGains(Constants.DriveConstants.Gains.kDefaultAzimuthGains)
            .driveCANId(Constants.RobotMap.frontLeftDrive)
            .aziCANId(Constants.RobotMap.frontLeftAzi)
            .aziEncoderCANId(Constants.RobotMap.frontLeftAzimuthEncoder)
            .offset(Constants.RobotMap.frontLeftOffset)
            .location(Constants.DriveConstants.frontLeftLocation)
            .build();

    public static final ModuleInfo frontRight =
        ModuleInfo.builder()
            .name(SwerveModules.FRONT_RIGHT)
            .driveGains(Constants.DriveConstants.Gains.kDefaultDrivingGains)
            .azimuthGains(Constants.DriveConstants.Gains.kDefaultAzimuthGains)
            .driveCANId(Constants.RobotMap.frontRightDrive)
            .aziCANId(Constants.RobotMap.frontRightAzi)
            .aziEncoderCANId(Constants.RobotMap.frontRightAzimuthEncoder)
            .offset(Constants.RobotMap.frontRightOffset)
            .location(Constants.DriveConstants.frontRightLocation)
            .build();

    public static final ModuleInfo backLeft =
        ModuleInfo.builder()
            .name(SwerveModules.BACK_LEFT)
            .driveGains(Constants.DriveConstants.Gains.kDefaultDrivingGains)
            .azimuthGains(Constants.DriveConstants.Gains.kDefaultAzimuthGains)
            .driveCANId(Constants.RobotMap.backLeftDrive)
            .aziCANId(Constants.RobotMap.backLeftAzi)
            .aziEncoderCANId(Constants.RobotMap.backLeftAzimuthEncoder)
            .offset(Constants.RobotMap.backLeftOffset)
            .location(Constants.DriveConstants.backLeftLocation)
            .build();

    public static final ModuleInfo backRight =
        ModuleInfo.builder()
            .name(SwerveModules.BACK_RIGHT)
            .driveGains(Constants.DriveConstants.Gains.kDefaultDrivingGains)
            .azimuthGains(Constants.DriveConstants.Gains.kDefaultAzimuthGains)
            .driveCANId(Constants.RobotMap.backRightDrive)
            .aziCANId(Constants.RobotMap.backRightAzi)
            .aziEncoderCANId(Constants.RobotMap.backRightAzimuthEncoder)
            .offset(Constants.RobotMap.backRightOffset)
            .location(Constants.DriveConstants.backRightLocation)
            .build();

    @UtilityClass
    public static final class Gains {
      public static final PIDFFGains kDefaultAzimuthGains =
          PIDFFGains.builder("BackRight/Default Azimuth").kP(0.65).tolerance(0).build();
      public static final PIDFFGains kDefaultDrivingGains =
          PIDFFGains.builder("BackRight/Default Driving").kP(1.0).kS(0.15).kV(2).build();
    }

    public static final PIDFFGains kFrontLeftAzimuthGains =
        PIDFFGains.builder("Front Left").kP(0.1).kS(0.12).tolerance(1.0).build();
    public static final PIDFFGains kFrontRightAzimuthGains =
        PIDFFGains.builder("Front Right").kP(0.1).kS(.12).tolerance(1.0).build();
    public static final PIDFFGains kBackLeftAzimuthGains =
        PIDFFGains.builder("Back Left").kP(0.1).kS(.15).tolerance(1.0).build();
    public static final PIDFFGains kBackRightAzimuthGains =
        PIDFFGains.builder("Back Right").kP(0.1).kS(.13).tolerance(1.0).build();
  }
}
