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
  public static final DebugMode DEBUG_MODE = DebugMode.MATCH;
  public static final int zero = 0; // in case you need a zero :)
  public static final int MOVE_FORWARD_SPEED = 5; // subject to move/change : )
  public static final double π = Math.PI;
  public static final double DOUBLE_PLACEHOLDER = zero;
  public static final int INT_PLACEHOLDER = zero;
  public static final boolean ENABLE_VISION_POSE_ESTIMATION = true;
  public static final double TUNE_MODULES_DRIVE_SPEED = Units.feetToMeters(3);
  public static final int CAN_TIMEOUT_MS = 200;

  @UtilityClass
  public static final class Logging {
    public static final String sda1Dir = "/media/sda1";
    public static final String sda2Dir = "/media/sda2";
  }

  public final class LimeLightConstants {
    public static double CAMERA_TO_TAG_MAX_DIST_INCHES = 120;
    public static double VISION_STD_DEVI_POSITION_IN_METERS = 0.9;
    public static double VISION_STD_DEVI_ROTATION_IN_RADIANS = Units.degreesToRadians(5);
    public static double MAX_POSE_JUMP_IN_INCHES = 6 * 12;
  }

  @UtilityClass
  public static final class RobotMap {
    public static final int PIGEON_CAN_ID = 20;

    public static final int ELEVATOR_LEFT_CANID = 7;
    public static final int ELEVATOR_RIGHT_CANID = 8;

    public static final int TOP_INTAKE_ROLLER = 12;
    public static final int BOTTOM_INTAKE_ROLLER = 11;

    // TODO: MAKE REAL!!!
    public static final int CONE_SLAPPER_MOTOR = 15;

    public static final int FOURBAR_ONE_CANID = 13;
    public static final int FOURBAR_TWO_CANID = 14;

    public static final int BLINKIN_PORT = 9;

    public static final int DRIVER_PORT = zero;
    public static final int OPERATOR_PORT = 1;
  }

  @UtilityClass
  public static class ElevatorConstants {
    public static final PIDFFGains ELEVATOR_GAINS =
        PIDFFGains.builder("Elevator Controller").kP(0.75).kD(0.0).kG(0.85).build();
    public static final double CARRIAGE_MASS_KG = Units.lbsToKilograms(12.0);
    public static final double ELEVATOR_DRUM_RADIUS_METERS = Units.inchesToMeters(1.0);
    public static final double ELEVATOR_MIN_HEIGHT_METERS = Units.inchesToMeters(0.0);
    public static final double ELEVATOR_MAX_HEIGHT_METERS = Units.inchesToMeters(50.0);
    public static final double ELEVATOR_MIN_HEIGHT_INCHES = 0.0;
    public static final double ELEVATOR_MAX_HEIGHT_INCHES = 50.0;
    public static final double ELEVATOR_PULLEY_DIAMETER = 2.0;
    public static final double ELEVATOR_GEAR_RATIO = 5.0;

    /*
     * manual calculations
     * 12 / 9.166701316833496 = 1.30908
     * 24 / 18.452327728271484 = 1.30064
     * 36 / 27.714082717895508 = 1.29897
     * 48 / 36.9520263671875 = 1.29898
     */
    public static final double ELEVATOR_POSITION_CONVERSION_FACTOR = 1.3019175;
    // (1.0 / ELEVATOR_GEAR_RATIO) * (Math.PI * ELEVATOR_PULLEY_DIAMETER);

    public static final double ELEVATOR_VELOCITY_CONVERSION_FACTOR =
        ELEVATOR_POSITION_CONVERSION_FACTOR / 60;
    public static final double ELEVATOR_ANGLE_DEGREES = 55.0;
    public static final int ELEVATOR_CURRENT_LIMIT = 30;

    // public static final double ELEVATOR_CONE_LOW_SCORE = 0;
    // public static final double ELEVATOR_CUBE_LOW_SCORE = 0;
    // public static final double ELEVATOR_CONE_MID_SCORE = 32.5;
    // public static final double ELEVATOR_CUBE_MID_SCORE = 12;
    // public static final double ELEVATOR_CONE_HIGH_SCORE = 50;
    // public static final double ELEVATOR_CUBE_HIGH_SCORE = 26;
  }

  @UtilityClass
  public static class FourBarConstants {
    public static final double MAX_ANGLE_DEGREES = -10;
    public static final double EXTENDED_ANGLE_DEGREES = 45;
    public static final double IDLE_ANGLE_DEGREES = 90;
    public static final double RETRACTED_ANGLE_DEGREES = 108.2;
    public static final double MAX_VELOCITY = 25;
    public static final double MAX_ACCELERATION = 75;
    public static final double MASS_KG = Units.lbsToKilograms(7.7);
    public static final double GEARING = 5 * 5 * 2.5;
    public static final double FOUR_BAR_ANGLE_CONVERSION = 1.0 / GEARING * 360;
    public static final double FOUR_BAR_VELOCITY_CONVERSION_FACTOR = FOUR_BAR_ANGLE_CONVERSION / 60;
    public static final int FOUR_BAR_BASE_CURRENT = 5;
    public static final PIDFFGains FOUR_BAR_CURRENT_GAINS =
        PIDFFGains.builder("4Bar Current Gains").kP(0.0).build();
    public static final int FOUR_BAR_MAX_CURRENT = 30;
    public static final double LENGTH_METRES = Units.inchesToMeters(10);
    public static final double HOMING_VOLTAGE = 0.25;
    public static final PIDFFGains FOUR_BAR_VOLTAGE_GAINS =
        PIDFFGains.builder("4Bar Controller").kP(10).kG(0.25).build();
  }

  @UtilityClass
  public static class IntakeConstants {
    public static final DCMotor INTAKE_MOTOR = DCMotor.getNeo550(1);
    public static final double BOTTOM_GEARING = 5.26;
    public static final double TOP_GEARING = 5.26;

    public static final double MAX_BOTTOM_RPM =
        Units.radiansPerSecondToRotationsPerMinute(INTAKE_MOTOR.freeSpeedRadPerSec)
            / BOTTOM_GEARING;
    public static final double MAX_TOP_RPM =
        Units.radiansPerSecondToRotationsPerMinute(INTAKE_MOTOR.freeSpeedRadPerSec) / TOP_GEARING;
    public static final double MOI = 0.00005;
    public static final int TOP_CURRENT_LIMIT = 20;
    public static final int BOTTOM_CURRENT_LIMIT = 20;
    public static final double TOP_POSITION_CONVERSION_FACTOR = 1 / TOP_GEARING;
    public static final double BOTTOM_POSITION_CONVERSION_FACTOR = 1 / BOTTOM_GEARING;
    public static final double TOP_VELOCITY_CONVERSION_FACTOR = 1 / TOP_GEARING;
    public static final double BOTTOM_VELOCITY_CONVERSION_FACTOR = 1 / BOTTOM_GEARING;
  }

  @UtilityClass
  public static class SlapperConstants {
    public static final DCMotor MOTOR = DCMotor.getNEO(1);
    public static final double GEARING = 48;
    public static final double LENGTH_METRES = Units.inchesToMeters(36);
    public static final PIDFFGains GAINS = PIDFFGains.builder("Slapper Gains").kP(0.5).build();

    // Degrees as measured on Unit Circle
    public static final double MIN_ANGLE_DEG = Constants.SlapperConstants.FULL_SEND_DEG;
    public static final double MAX_ANGLE_DEG = Constants.SlapperConstants.RESTING_DEG;

    public static final double FULL_SEND_DEG = 8;
    public static final double RESTING_DEG = 90 + 15.5;

    public static final double MOI = 0.5;
    public static final int CURRENT_LIMIT = 20;
    public static final double POSITION_CONVERSION_FACTOR = 1 / GEARING;
    public static final double VELOCITY_CONVERSION_FACTOR = 1 / GEARING;
  }

  @UtilityClass
  public static final class DriveConstants {

    @UtilityClass
    public static class FieldTunables {
      // OTF Trajctory Generation (go over or under Charge Station, around barrier)
      public static final double MIN_GO_TOP = 4;
      public static final double MAX_GO_TOP = 6;
      public static final double MAX_GO_BOTTOM = MIN_GO_TOP - 1;
      public static final double MIN_GO_BOTTOM = 2;
      public static final double TIME_BETWEEN_REGERATION_SECONDS = 3;

      public static final double CHARGE_STATION_OFFSET = 0.8;

      public static final double SINGLE_HUMAN_STATION_OFFSET = 0.6;

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
    public static final double WHEEL_DIAMETER = 3.9; // 4.02267; // 3.85;
    public static final double GEAR_RATIO = 6.12;
    public static final double DIST_PER_PULSE =
        (1.0 / GEAR_RATIO) * Units.inchesToMeters(WHEEL_DIAMETER) * Math.PI;
    // 1;
    public static final double MAX_SWERVE_VEL = Units.feetToMeters(6.0);
    public static final double MAX_SWERVE_VEL_AUTO = Units.feetToMeters(12.0);
    public static final double MAX_SWERVE_AZI = Math.PI;
    public static final double MAX_SWERVE_ACCEL = Units.feetToMeters(5);
    public static final double MAX_ROTATIONAL_SPEED_RAD_PER_SEC = Units.degreesToRadians(275);

    public static final int DRIVE_CURRENT_LIMIT = 50;
    public static final int AZI_CURRENT_LIMIT = 20;

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
        PIDFFGains.builder("Heading Controller").kP(12).kS(3).kD(0.35).tolerance(1).build();

    public static final PIDFFGains K_BRIDGE_CONTROLLER_GAINS =
        PIDFFGains.builder("Bridge Controller").kP(0.01).kD(0).tolerance(zero).build();

    public static final ModuleInfo FRONT_LEFT =
        ModuleInfo.builder()
            .name(SwerveModuleName.FRONT_LEFT)
            .driveGains(Constants.DriveConstants.Gains.K_DEFAULT_DRIVING_GAINS)
            .azimuthGains(Constants.DriveConstants.Gains.K_DEFAULT_AZIMUTH_GAINS)
            .driveCANId(1)
            .aziCANId(2)
            .aziEncoderCANId(zero)
            .offset(0.312)
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
            .offset(0.857)
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
            .offset(0.864)
            .location(BACK_RIGHT_LOCATION)
            .build();

    @UtilityClass
    public static final class Gains {
      public static final PIDFFGains K_DEFAULT_AZIMUTH_GAINS =
          PIDFFGains.builder("BackRight/Default Azimuth").kP(0.12).tolerance(0.75).build();
      public static final PIDFFGains K_DEFAULT_DRIVING_GAINS =
          PIDFFGains.builder("BackRight/Default Driving").kP(1).kD(0).kS(0.225).kV(2.33).build();

      public static final PIDFFGains K_TRAJECTORY_CONTROLLER_GAINS_X =
          PIDFFGains.builder("Trajectory Controller X-Axis").kP(7).kD(0.0).build();

      public static final PIDFFGains K_TRAJECTORY_CONTROLLER_GAINS_Y =
          PIDFFGains.builder("Trajectory Controller Y-Axis").kP(7).kD(0.0).build();

      public static final PIDFFGains K_TRAJECTORY_CONTROLLER_GAINS_ROTATION =
          PIDFFGains.builder("Trajectory Controller Rotation").kP(2.5).kD(0.0).build();
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

    public static final SuperstructureConfig HOLD_CONE =
        SuperstructureConfig.builder().topRPM(500).bottomRPM(-500).build();

    public static final SuperstructureConfig HOLD_CUBE =
        SuperstructureConfig.builder().topRPM(-100).bottomRPM(100).build();

    public static final SuperstructureConfig INTAKE_TIPPED_CONE =
        SuperstructureConfig.builder()
            .elevatorPosition(1.5)
            .fourBarPosition(-28)
            .topRPM(1250)
            .bottomRPM(-1250)
            .build();
    public static final SuperstructureConfig INTAKE_UPRIGHT_CONE =
        SuperstructureConfig.builder()
            .elevatorPosition(0)
            .fourBarPosition(29)
            .topRPM(1250)
            .bottomRPM(-1250)
            .build();

    public static final SuperstructureConfig INTAKE_SHELF_CONE =
        SuperstructureConfig.builder()
            .elevatorPosition(42.875)
            .fourBarPosition(50)
            .topRPM(1250)
            .bottomRPM(-1250)
            .build();

    public static final SuperstructureConfig INTAKE_CUBE =
        SuperstructureConfig.builder()
            .elevatorPosition(0)
            .fourBarPosition(17)
            .topRPM(1_500)
            .bottomRPM(1_500)
            .build();

    public static final SuperstructureConfig INTAKE_CUBE_DEFLATED =
        SuperstructureConfig.builder()
            .elevatorPosition(0)
            .fourBarPosition(20)
            .topRPM(1_500)
            .bottomRPM(1_500)
            .build();

    public static final SuperstructureConfig SCORE_CUBE_LOW =
        SuperstructureConfig.builder()
            .elevatorPosition(0)
            .fourBarPosition(24)
            .topRPM(-1000)
            .bottomRPM(-1000)
            .build();

    public static final SuperstructureConfig SCORE_CUBE_MID =
        SuperstructureConfig.builder()
            .elevatorPosition(15)
            .fourBarPosition(90)
            .topRPM(1350)
            .bottomRPM(1350)
            .build();

    public static final SuperstructureConfig SCORE_CUBE_HIGH =
        SuperstructureConfig.builder()
            .elevatorPosition(32)
            .fourBarPosition(75)
            .topRPM(1750)
            .bottomRPM(1750)
            .build();

    public static final SuperstructureConfig SCORE_CONE_LOW =
        SuperstructureConfig.builder()
            .elevatorPosition(0)
            .fourBarPosition(45)
            .topRPM(-1000)
            .bottomRPM(-1000)
            .build();

    public static final SuperstructureConfig SCORE_CONE_MID =
        SuperstructureConfig.builder()
            .elevatorPosition(28)
            .fourBarPosition(80)
            .topRPM(-500)
            .bottomRPM(500)
            .build();

    public static final SuperstructureConfig SCORE_CONE_HIGH =
        SuperstructureConfig.builder()
            .elevatorPosition(48)
            .fourBarPosition(40)
            .topRPM(-1000)
            .bottomRPM(-1000)
            .build();
  }
}
