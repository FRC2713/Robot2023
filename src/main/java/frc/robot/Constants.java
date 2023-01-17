// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
// liam sais hi :)
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.SwerveIO.module.ModuleInfo;
import frc.robot.subsystems.SwerveIO.module.SwerveModuleName;
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

    public static final int BLINKIN_PORT = 1000;
  }

  public static class Elevator {
    public static final double CARRIAGE_MASS_KG = Units.lbsToKilograms(20.0);
    public static final double ELEVATOR_DRUM_RADIUS_METERS = Units.inchesToMeters(1.0);
    public static final double ELEVATOR_MIN_HEIGHT_METERS = Units.inchesToMeters(0.0);
    public static final double ELEVATOR_MAX_HEIGHT_METERS = Units.inchesToMeters(40.0);
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
            .name(SwerveModuleName.FRONT_LEFT)
            .driveGains(Constants.DriveConstants.Gains.kDefaultDrivingGains)
            .azimuthGains(Constants.DriveConstants.Gains.kDefaultAzimuthGains)
            .driveCANId(1)
            .aziCANId(8)
            .aziEncoderCANId(zero)
            .offset(0.1124)
            .location(frontLeftLocation)
            .build();

    public static final ModuleInfo frontRight =
        ModuleInfo.builder()
            .name(SwerveModuleName.FRONT_RIGHT)
            .driveGains(Constants.DriveConstants.Gains.kDefaultDrivingGains)
            .azimuthGains(Constants.DriveConstants.Gains.kDefaultAzimuthGains)
            .driveCANId(3)
            .aziCANId(2)
            .aziEncoderCANId(1)
            .offset(0.6028)
            .location(frontRightLocation)
            .build();

    public static final ModuleInfo backLeft =
        ModuleInfo.builder()
            .name(SwerveModuleName.BACK_LEFT)
            .driveGains(Constants.DriveConstants.Gains.kDefaultDrivingGains)
            .azimuthGains(Constants.DriveConstants.Gains.kDefaultAzimuthGains)
            .driveCANId(4)
            .aziCANId(5)
            .aziEncoderCANId(2)
            .offset(0.0626)
            .location(backLeftLocation)
            .build();

    public static final ModuleInfo backRight =
        ModuleInfo.builder()
            .name(SwerveModuleName.BACK_RIGHT)
            .driveGains(Constants.DriveConstants.Gains.kDefaultDrivingGains)
            .azimuthGains(Constants.DriveConstants.Gains.kDefaultAzimuthGains)
            .driveCANId(6)
            .aziCANId(7)
            .aziEncoderCANId(3)
            .offset(0.775)
            .location(backRightLocation)
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
