// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.swerveIO.module.ModuleInfo;
import frc.robot.subsystems.swerveIO.module.SwerveModuleName;
import frc.robot.util.DebugMode;
import frc.robot.util.FieldConstants;
import frc.robot.util.PIDFFGains;
import frc.robot.util.SuperstructureConfig;
import lombok.experimental.UtilityClass;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
@UtilityClass
public final class Constants {

  public static final boolean TUNING_MODE = false;
  public static final DebugMode DEBUG_MODE = DebugMode.TUNE_MODULES;
  public static final int zero = 0; // in case you need a zero :)
  public static final int MOVE_FORWARD_SPEED = 5; // subject to move/change : )
  public static final double π = Math.PI;
  public static final double DOUBLE_PLACEHOLDER = zero;
  public static final int INT_PLACEHOLDER = zero;
  public static final boolean ENABLE_VISION_POSE_ESTIMATION = false;
  public static final double TUNE_MODULES_DRIVE_SPEED = Units.feetToMeters(3);

  @UtilityClass
  public static final class RobotMap {
    public static final int PIGEON_CAN_ID = 20;

    public static final int ELEVATOR_LEFT_CANID = 7;
    public static final int ELEVATOR_RIGHT_CANID = 8;

    public static final int INTAKE_WHEELS_CANID = 12;
    public static final int INTAKE_ROLLERS_CANID = 11;

    public static final int FOURBAR_ONE_CANID = 13;
    public static final int FOURBAR_TWO_CANID = 14;

    public static final int BLINKIN_PORT = 9;

    public static final int DRIVER_PORT = zero;
    public static final int OPERATOR_PORT = 1;
  }

  @UtilityClass
  public static class ElevatorConstants {
    public static final PIDFFGains ELEVATOR_GAINS =
        PIDFFGains.builder("Elevator Controller").kP(1).kD(0.0).kG(0.54).build();
    public static final double CARRIAGE_MASS_KG = Units.lbsToKilograms(12.0);
    public static final double ELEVATOR_DRUM_RADIUS_METERS = Units.inchesToMeters(1.0);
    public static final double ELEVATOR_MIN_HEIGHT_METERS = Units.inchesToMeters(0.0);
    public static final double ELEVATOR_MAX_HEIGHT_METERS = Units.inchesToMeters(50.0);
    public static final double ELEVATOR_PULLEY_DIAMETER = 2.0;
    public static final double ELEVATOR_GEAR_RATIO = 5.0;

    /* manual calculations
       12 / 9.166701316833496 = 1.30908
       24 / 18.452327728271484 = 1.30064
       36 / 27.714082717895508 = 1.29897
       48 / 36.9520263671875 = 1.29898
    */
    public static final double ELEVATOR_POSITION_CONVERSION_FACTOR = 1.3019175;
    // (1.0 / ELEVATOR_GEAR_RATIO) * (Math.PI * ELEVATOR_PULLEY_DIAMETER);

    public static final double ELEVATOR_VELOCITY_CONVERSION_FACTOR =
        ELEVATOR_POSITION_CONVERSION_FACTOR / 60;
    public static final double ELEVATOR_ANGLE_DEGREES = 55.0;
    public static final int ELEVATOR_CURRENT_LIMIT = 45;

    public static final double ELEVATOR_CONE_LOW_SCORE = 0;
    public static final double ELEVATOR_CUBE_LOW_SCORE = 0;
    public static final double ELEVATOR_CONE_MID_SCORE = 35;
    public static final double ELEVATOR_CUBE_MID_SCORE = 17;
    public static final double ELEVATOR_CONE_HIGH_SCORE = 50;
    public static final double ELEVATOR_CUBE_HIGH_SCORE = 30;

    public static final double ELEVATOR_CUBE_FLOOR_INTAKE = 8;

    public static final double ELEVATOR_CONE_FLOOR_TIPPED_INTAKE = zero;

    public static final double ELEVATOR_CONE_FLOOR_UP_INTAKE = 12;
  }

  @UtilityClass
  public static class FourBarConstants {
    public static final double MAX_ANGLE_RADIANS = Units.degreesToRadians(-10);
    public static final double EXTENDED_ANGLE_RADIANS = Units.degreesToRadians(45);
    public static final double IDLE_ANGLE_RADIANS = Units.degreesToRadians(90);
    public static final double RETRACTED_ANGLE_RADIANS = Units.degreesToRadians(117.5);
    public static final double MAX_VELOCITY = 1600;
    public static final double MAX_ACCELERATION = 5000;
    public static final double MASS_KG = Units.lbsToKilograms(7.7);
    public static final double GEARING = 250.0;
    public static final double FOUR_BAR_ANGLE_CONVERSION = 1.0 / GEARING * 360;
    public static final double FOUR_BAR_VELOCITY_CONVERSION_FACTOR = FOUR_BAR_ANGLE_CONVERSION / 60;
    public static final int FOUR_BAR_CURRENT_LIMIT = 50;
    public static final double LENGTH_METRES = Units.inchesToMeters(10);
    public static final PIDFFGains FOUR_BAR_GAINS =
        PIDFFGains.builder("4Bar Controller").kP(0.25).kI(zero).kD(zero).kG(0.000).build();
  }

  @UtilityClass
  public static class IntakeConstants {
    public static final DCMotor INTAKE_MOTOR = DCMotor.getNEO(1);
    public static final double ROLLER_GEARING = 24.0 / 20.0;
    public static final double WHEELS_GEARING = 1;

    public static final double MAX_ROLLER_RPM =
        Units.radiansPerSecondToRotationsPerMinute(INTAKE_MOTOR.freeSpeedRadPerSec)
            / ROLLER_GEARING;
    public static final double MAX_WHEEL_RPM =
        Units.radiansPerSecondToRotationsPerMinute(INTAKE_MOTOR.freeSpeedRadPerSec)
            / WHEELS_GEARING;
    public static final double MOI = 0.1;
    public static final int WHEELS_CURRENT_LIMIT = 50;
    public static final int ROLLERS_CURRENT_LIMIT = 50;
    public static final double WHEELS_POSITION_CONVERSION_FACTOR = 1; // SUBJECT TO CHANGE
    public static final double ROLLERS_POSITION_CONVERSION_FACTOR = 1; // SUBJECT TO CHANGE
    public static final double WHEELS_VELOCITY_CONVERSION_FACTOR = 1; // SUBJECT TO CHANGE
    public static final double ROLLERS_VELOCITY_CONVERSION_FACTOR = 1; // SUBJECT TO CHANGE
    public static final double ROLLERS_CONE_TIPPED_INTAKE_RPM = 100;
    public static final double ROLLERS_CONE_UPRIGHT_INTAKE_RPM = 100;
    public static final double ROLLERS_CUBE_INTAKE_RPM = 1500;
    public static final double ROLLERS_CONE_SCORE_RPM = -100;
    public static final double ROLLERS_CUBE_SCORE_RPM = -100;
    public static final double WHEELS_CONE_TIPPED_INTAKE_RPM = 100;
    public static final double WHEELS_CONE_UPRIGHT_INTAKE_RPM = 100;
    public static final double WHEELS_CUBE_INTAKE_RPM = 0;
    public static final double WHEELS_CONE_SCORE_RPM = -100;
    public static final double WHEELS_CUBE_SCORE_RPM = -100;
  }

  @UtilityClass
  public static final class DriveConstants {

    @UtilityClass
    public static class FieldTunables {
      // OTF Trajctory Generation (go over or under Charge Station)
      public static final double MIN_GO_TOP = 4;
      public static final double MAX_GO_TOP = 6;
      public static final double MAX_GO_BOTTOM = MIN_GO_TOP - 1;
      public static final double MIN_GO_BOTTOM = 2;
      public static final double TIME_BETWEEN_REGERATION_SECONDS = 3;

      public static final double CHARGE_STATION_OFFSET = 0.6;

      public static final double GRID_OFFSET = 0.7;

      public static final Rotation2d CLOSEST_GRID_HEADING = Rotation2d.fromDegrees(180);

      public static final PathPoint GRID_CORNERS_FOR_SWERVE[] = {
        // Top Left
        new PathPoint(
            FieldConstants.Community.chargingStationCorners[1].plus(
                new Translation2d(0, Constants.DriveConstants.FieldTunables.CHARGE_STATION_OFFSET)),
            CLOSEST_GRID_HEADING,
            CLOSEST_GRID_HEADING),
        // Top Right
        new PathPoint(
            FieldConstants.Community.chargingStationCorners[3].plus(
                new Translation2d(0, Constants.DriveConstants.FieldTunables.CHARGE_STATION_OFFSET)),
            CLOSEST_GRID_HEADING,
            CLOSEST_GRID_HEADING),

        // Bottom Left
        new PathPoint(
            FieldConstants.Community.chargingStationCorners[0].minus(
                new Translation2d(0, Constants.DriveConstants.FieldTunables.CHARGE_STATION_OFFSET)),
            CLOSEST_GRID_HEADING,
            CLOSEST_GRID_HEADING),

        // Bottom Right
        new PathPoint(
            FieldConstants.Community.chargingStationCorners[2].minus(
                new Translation2d(0, Constants.DriveConstants.FieldTunables.CHARGE_STATION_OFFSET)),
            CLOSEST_GRID_HEADING,
            CLOSEST_GRID_HEADING),
      };
    }

    public static final double K_JOYSTICK_TURN_DEADZONE = 0.04;
    public static final double WHEEL_DIAMETER = 4;
    public static final double GEAR_RATIO = 6.12;
    public static final double DIST_PER_PULSE =
        (1.0 / GEAR_RATIO) * Units.inchesToMeters(WHEEL_DIAMETER) * Math.PI;

    public static final double MAX_SWERVE_VEL = Units.feetToMeters(16.0 * 0.75);
    public static final double MAX_SWERVE_AZI = Math.PI;
    public static final double MAX_SWERVE_ACCEL = Units.feetToMeters(1);
    public static final double MAX_ROTATIONAL_SPEED_RAD_PER_SEC = Units.degreesToRadians(180);

    public static final int CURRENT_LIMIT = 25;

    public static final double K_MODULE_DISTANCE_FROM_CENTER = Units.inchesToMeters(20.75 / 2);

    private static final Translation2d FRONT_LEFT_LOCATION =
        new Translation2d(
            DriveConstants.K_MODULE_DISTANCE_FROM_CENTER,
            DriveConstants.K_MODULE_DISTANCE_FROM_CENTER);
    private static final Translation2d FRONT_RIGHT_LOCATION =
        new Translation2d(
            DriveConstants.K_MODULE_DISTANCE_FROM_CENTER,
            -DriveConstants.K_MODULE_DISTANCE_FROM_CENTER);
    private static final Translation2d BACK_LEFT_LOCATION =
        new Translation2d(
            -DriveConstants.K_MODULE_DISTANCE_FROM_CENTER,
            DriveConstants.K_MODULE_DISTANCE_FROM_CENTER);
    private static final Translation2d BACK_RIGHT_LOCATION =
        new Translation2d(
            -DriveConstants.K_MODULE_DISTANCE_FROM_CENTER,
            -DriveConstants.K_MODULE_DISTANCE_FROM_CENTER);

    public static final SwerveDriveKinematics KINEMATICS =
        new SwerveDriveKinematics(
            FRONT_LEFT_LOCATION, FRONT_RIGHT_LOCATION, BACK_LEFT_LOCATION, BACK_RIGHT_LOCATION);

    private static final double BUMPERLESS_ROBOT_LENGTH = Units.inchesToMeters(26.5);
    private static final double BUMPERLESS_ROBOT_WIDTH = Units.inchesToMeters(26.5);
    private static final double BUMPER_THICKNESS = Units.inchesToMeters(3);

    public static final double FULL_ROBOT_WIDTH = BUMPERLESS_ROBOT_WIDTH + BUMPER_THICKNESS * 2;
    public static final double FULL_ROBOT_LENGTH = BUMPERLESS_ROBOT_LENGTH + BUMPER_THICKNESS * 2;

    public static final double HEADING_CONTROLLER_DRIVER_CHANGE_RATE = 4;
    public static final PIDFFGains K_HEADING_CONTROLLER_GAINS =
        PIDFFGains.builder("Heading Controller").kP(1).kD(0.01).tolerance(zero).build();

    public static final ModuleInfo FRONT_LEFT =
        ModuleInfo.builder()
            .name(SwerveModuleName.FRONT_LEFT)
            .driveGains(Constants.DriveConstants.Gains.K_DEFAULT_DRIVING_GAINS)
            .azimuthGains(Constants.DriveConstants.Gains.K_DEFAULT_AZIMUTH_GAINS)
            .driveCANId(1)
            .aziCANId(2)
            .aziEncoderCANId(zero)
            .offset(0.317)
            .location(FRONT_LEFT_LOCATION)
            .build();

    public static final ModuleInfo FRONT_RIGHT =
        ModuleInfo.builder()
            .name(SwerveModuleName.FRONT_RIGHT)
            .driveGains(Constants.DriveConstants.Gains.K_DEFAULT_DRIVING_GAINS)
            .azimuthGains(Constants.DriveConstants.Gains.K_DEFAULT_AZIMUTH_GAINS)
            .driveCANId(3)
            .aziCANId(4)
            .aziEncoderCANId(1)
            .offset(0.527)
            .location(FRONT_RIGHT_LOCATION)
            .build();

    public static final ModuleInfo BACK_LEFT =
        ModuleInfo.builder()
            .name(SwerveModuleName.BACK_LEFT)
            .driveGains(Constants.DriveConstants.Gains.K_DEFAULT_DRIVING_GAINS)
            .azimuthGains(Constants.DriveConstants.Gains.K_DEFAULT_AZIMUTH_GAINS)
            .driveCANId(10)
            .aziCANId(9)
            .aziEncoderCANId(2)
            .offset(0.86)
            .location(BACK_LEFT_LOCATION)
            .build();

    public static final ModuleInfo BACK_RIGHT =
        ModuleInfo.builder()
            .name(SwerveModuleName.BACK_RIGHT)
            .driveGains(Constants.DriveConstants.Gains.K_DEFAULT_DRIVING_GAINS)
            .azimuthGains(Constants.DriveConstants.Gains.K_DEFAULT_AZIMUTH_GAINS)
            .driveCANId(5)
            .aziCANId(6)
            .aziEncoderCANId(3)
            .offset(0.851)
            .location(BACK_RIGHT_LOCATION)
            .build();

    @UtilityClass
    public static final class Gains {
      public static final PIDFFGains K_DEFAULT_AZIMUTH_GAINS =
          PIDFFGains.builder("BackRight/Default Azimuth").kP(0.07).tolerance(0).build();
      public static final PIDFFGains K_DEFAULT_DRIVING_GAINS =
          PIDFFGains.builder("BackRight/Default Driving").kP(1.0).kS(0.15).kV(2).build();

      public static final PIDFFGains K_TRAJECTORY_CONTROLLER_GAINS_X =
          PIDFFGains.builder("Trajectory Controller X-Axis").kP(0.9).kD(0.0).build();

      public static final PIDFFGains K_TRAJECTORY_CONTROLLER_GAINS_Y =
          PIDFFGains.builder("Trajectory Controller Y-Axis").kP(0.9).kD(0.0).build();

      public static final PIDFFGains K_TRAJECTORY_CONTROLLER_GAINS_ROTATION =
          PIDFFGains.builder("Trajectory Controller Rotation").kP(1.0).kD(0.0).build();
    }

    public static final PIDFFGains K_FRONT_LEFT_AZIMUTH_GAINS =
        PIDFFGains.builder("Front Left").kP(0.1).kS(0.12).tolerance(1.0).build();
    public static final PIDFFGains K_FRONT_RIGHT_AZIMUTH_GAINS =
        PIDFFGains.builder("Front Right").kP(0.1).kS(.12).tolerance(1.0).build();
    public static final PIDFFGains K_BACK_LEFT_AZIMUTH_GAINS =
        PIDFFGains.builder("Back Left").kP(0.1).kS(.15).tolerance(1.0).build();
    public static final PIDFFGains K_BACK_RIGHT_AZIMUTH_GAINS =
        PIDFFGains.builder("Back Right").kP(0.1).kS(.13).tolerance(1.0).build();
  }

  public static final class SuperstructureConstants {
    public static final SuperstructureConfig INTAKE_TIPPED_CONE =
        SuperstructureConfig.builder()
            .elevatorPosition(0)
            .fourBarPosition(-20)
            .wheelRPM(-3000)
            .rollerRPM(-3000)
            .build();
    public static final SuperstructureConfig INTAKE_UPRIGHT_CONE =
        SuperstructureConfig.builder()
            .elevatorPosition(0)
            .fourBarPosition(45)
            .wheelRPM(-3000)
            .rollerRPM(-3000)
            .build();
    public static final SuperstructureConfig INTAKE_CUBE =
        SuperstructureConfig.builder()
            .elevatorPosition(0)
            .fourBarPosition(20)
            .wheelRPM(-1500)
            .rollerRPM(1500)
            .build();
    public static final SuperstructureConfig SCORE =
        SuperstructureConfig.builder().fourBarPosition(0).wheelRPM(1000).rollerRPM(1000).build();
    // public static final SuperstructureConfig SCORE_CUBE =
    //     SuperstructureConfig.builder().elevatorPosition(1).fourBarPosition(1)
    //     .wheelRPM(1000)
    //     .rollerRPM(1000).build();
  }
}
