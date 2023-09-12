package frc.robot.util;

import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;
import com.pathplanner.lib.PathPoint;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.REVLibError;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Robot.GamePieceMode;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import lombok.NonNull;
import lombok.experimental.UtilityClass;
import org.littletonrobotics.junction.Logger;

@UtilityClass
public final class RedHawkUtil {

  /**
   * Checks whether the given REVLibError is actually an error, and then logs it to AdvantageScope
   * and SmartDasboard. SmartDashboard variable logged is "RevLibError" and "RevLibError/name"
   * AdvantageScope variable logged is "RevLibError/name"
   *
   * @param status A RevLibError
   * @param name The name of the RevLibError, logged (see description)
   */
  public static void errorHandleSparkMAX(@NonNull REVLibError status) {
    if (status != REVLibError.kOk) {
      StackTraceElement[] rawStackTrace = Thread.currentThread().getStackTrace();
      ErrHandler.getInstance()
          .addError(
              status.name()
                  + " StackTrace: "
                  + rawStackTrace[2].getFileName()
                  + ":"
                  + rawStackTrace[2].getLineNumber());
    }
  }

  public static void cOk(REVLibError status) {
    errorHandleSparkMAX(status);
  }

  public static Translation2d Pose2dToTranslation2d(Pose2d pose) {
    return new Translation2d(pose.getX(), pose.getY());
  }

  /**
   * Checkes if given pose if past the mid point of the field form their community (exclusive).
   * Flips {@code pose} if on red alliance
   *
   * @param pose the pose to check
   */
  public static boolean pastMidPoint(Pose2d pose) {
    return Reflections.reflectIfRed(pose.getX()) > (FieldConstants.fieldLength / 2);
  }

  public static PathPoint currentPositionPathPoint(Rotation2d heading) {
    return new PathPoint(
        RedHawkUtil.Pose2dToTranslation2d(Robot.swerveDrive.getUsablePose()),
        heading,
        Robot.swerveDrive.getUsablePose().getRotation(),
        Robot.swerveDrive.getAverageVelocity());
  }

  public static Translation2d getClosestGrid(double y) {
    return Arrays.asList(
            Robot.gamePieceMode == GamePieceMode.CUBE
                ? FieldConstants.Grids.cubeComplexLowTranslations
                : FieldConstants.Grids.coneComplexLowTranslations)
        .stream()
        .sorted(
            (a, b) ->
                Double.compare(
                    a.getDistance(Robot.swerveDrive.getUsablePose().getTranslation()),
                    b.getDistance(Robot.swerveDrive.getUsablePose().getTranslation())))
        .findFirst()
        .get()
        .plus(new Translation2d(Constants.DriveConstants.FieldTunables.GRID_OFFSET, 0));
  }

  public static int getClosestGridNumber(double y) {
    return Arrays.asList(FieldConstants.Grids.complexLowTranslations)
        .indexOf(
            Arrays.asList(
                    Robot.gamePieceMode == GamePieceMode.CUBE
                        ? FieldConstants.Grids.cubeComplexLowTranslations
                        : FieldConstants.Grids.coneComplexLowTranslations)
                .stream()
                .sorted(
                    (a, b) ->
                        Double.compare(
                            a.getDistance(Robot.swerveDrive.getUsablePose().getTranslation()),
                            b.getDistance(Robot.swerveDrive.getUsablePose().getTranslation())))
                .findFirst()
                .get());
  }

  public static boolean isOnChargeStation(Translation2d location) {
    // TODO: Get working on red alliance
    double x = location.getX();
    double y = location.getY();
    return (x < FieldConstants.Community.chargingStationOuterX
        && x > FieldConstants.Community.chargingStationInnerX
        && y < FieldConstants.Community.chargingStationLeftY
        && y > FieldConstants.Community.chargingStationRightY);
  }

  public static boolean isOnChargeStation(Pose2d location) {
    return isOnChargeStation(Pose2dToTranslation2d(location));
  }

  public static class ErrHandler {
    private static ErrHandler INSTANCE;

    /**
     * Gets the instance of the ErrHandler singleton
     *
     * @return The one instance of ErrHandler
     */
    public static ErrHandler getInstance() {
      if (INSTANCE == null) {
        INSTANCE = new ErrHandler();
      }
      return INSTANCE;
    }

    private ErrHandler() {}

    private List<String> errors = new ArrayList<>();

    /**
     * Adds an error to the list of errors. Must be called on the singleton (see {@code
     * getInstance}). Also logs.
     */
    public void addError(@NonNull String error) {
      this.errors.add(error);
      this.log();
    }

    public void log() {
      Logger.getInstance().recordOutput("Errors", String.join(" \\\\ ", errors));
    }
  }

  public static Twist2d poseLog(final Pose2d transform) {
    final double kEps = 1E-9;

    final double dtheta = transform.getRotation().getRadians();
    final double half_dtheta = 0.5 * dtheta;
    final double cos_minus_one = transform.getRotation().getCos() - 1.0;
    double halftheta_by_tan_of_halfdtheta;
    if (Math.abs(cos_minus_one) < kEps) {
      halftheta_by_tan_of_halfdtheta = 1.0 - 1.0 / 12.0 * dtheta * dtheta;
    } else {
      halftheta_by_tan_of_halfdtheta =
          -(half_dtheta * transform.getRotation().getSin()) / cos_minus_one;
    }
    final Translation2d translation_part =
        transform
            .getTranslation()
            .rotateBy(new Rotation2d(halftheta_by_tan_of_halfdtheta, -half_dtheta));
    return new Twist2d(translation_part.getX(), translation_part.getY(), dtheta);
  }

  public static class Reflections {
    public static Translation2d reflectIfRed(Translation2d old) {
      if (DriverStation.getAlliance() == Alliance.Red) {
        return reflect(old);
      }
      return old;
    }

    public static Translation2d reflectIfBlue(Translation2d old) {
      if (DriverStation.getAlliance() == Alliance.Blue) {
        return reflect(old);
      }
      return old;
    }

    public static Translation2d reflect(Translation2d old) {
      return new Translation2d(FieldConstants.fieldLength - old.getX(), old.getY());
    }

    public static double reflectIfRed(double x) {
      return reflectIfRed(new Translation2d(x, 0)).getX();
    }

    public static Rotation2d reflectIfRed(Rotation2d old) {
      if (DriverStation.getAlliance() == Alliance.Red) {
        return old.minus(Rotation2d.fromDegrees(180));
      }
      return old;
    }

    public static Pose2d reflectIfRed(Pose2d old) {
      return new Pose2d(reflectIfRed(old.getTranslation()), reflectIfRed(old.getRotation()));
    }
  }

  public static void configureCANSparkMAXStatusFrames(
      HashMap<CANSparkMaxLowLevel.PeriodicFrame, Integer> config, CANSparkMax... sparks) {
    config.forEach(
        (frame, ms) -> {
          for (CANSparkMax spark : sparks) {
            cOk(spark.setPeriodicFramePeriod(frame, ms));
          }
        });
  }

  public static void configurePigeonStatusFrames(
      Pigeon2 pigeon, HashMap<PigeonIMU_StatusFrame, Integer> config) {
    config.forEach(
        (frame, ms) -> {
          pigeon.setStatusFramePeriod(frame, ms);
        });
  }

  // // https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces
  // public static void configureDefaultTrafficSpark(CANSparkMax spark) {
  //   // Applied output, faults, sticky faults, isFollower
  //   spark.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10);

  //   // velocity, temperature, voltage, current
  //   spark.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);

  //   // position
  //   spark.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);

  //   // analog sensor voltage, analog sensor velocity, analog sensor position
  //   spark.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 50);

  //   // alt encoder velocity, alt encoder position
  //   spark.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 20);

  //   // duty cycle absolute encoder position, duty cycle absolute encoder angle
  //   spark.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 200);

  //   // duty cycle absolute encoder velocity, duty cycle absolute encoder frequency
  //   spark.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 200);
  // }

  // // https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces
  // // Used for sparks that aren't that important and don't need to be broadcasting info very often
  // public static void configureLowTrafficSpark(CANSparkMax spark) {
  //   // Applied output, faults, sticky faults, isFollower
  //   spark.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);

  //   // velocity, temperature, voltage, current
  //   spark.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 40);

  //   // position
  //   spark.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 1000);

  //   // analog sensor voltage, analog sensor velocity, analog sensor position
  //   spark.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 10000);

  //   // alt encoder velocity, alt encoder position
  //   spark.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 10000);

  //   // duty cycle absolute encoder position, duty cycle absolute encoder angle
  //   spark.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 10000);

  //   // duty cycle absolute encoder velocity, duty cycle absolute encoder frequency
  //   spark.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 10000);
  //   configureDefaultTrafficSpark(spark);
  // }

  // // https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces
  // // Used for sparks that are critical to robot functionality
  // public static void configureHighTrafficSpark(CANSparkMax spark) {
  //   // Applied output, faults, sticky faults, isFollower
  //   spark.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 1000);

  //   // velocity, temperature, voltage, current
  //   spark.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 10);

  //   // position
  //   spark.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);

  //   // analog sensor voltage, analog sensor velocity, analog sensor position
  //   spark.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 10000);

  //   // alt encoder velocity, alt encoder position
  //   spark.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 10000);

  //   // duty cycle absolute encoder position, duty cycle absolute encoder angle
  //   // spark.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 10000);

  //   // duty cycle absolute encoder velocity, duty cycle absolute encoder frequency
  //   spark.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 10000);
  //   configureDefaultTrafficSpark(spark);
  // }

  // // https://v5.docs.ctr-electronics.com/en/stable/ch18_CommonAPI.html
  // public static void configureDefaultPigeon2(Pigeon2 pigeon) {
  //   // 5, 7, 8 don't seem to exist

  //   // calibration status, IMU temperature
  //   pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_1_General, 100);

  //   // biased gyro values (x, y, z)
  //   pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_2_GeneralCompass, 100);

  //   // accelerometer derived angles
  //   pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_3_GeneralAccel, 100);

  //   // unprocessed magnetometer values (x, y, z)
  //   pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.BiasedStatus_4_Mag, 20);

  //   // 9 degree fused yaw, pitch, roll (requires magnetometer calibration)
  //   pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.BiasedStatus_6_Accel, 10);

  //   // six degree fused yaw, pitch, roll
  //   pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR, 10);

  //   // six degree fused quaternion
  //   pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_10_SixDeg_Quat, 100);

  //   // accumulated gyro angles
  //   pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_11_GyroAccum, 20);
  // }

  // // https://v5.docs.ctr-electronics.com/en/stable/ch18_CommonAPI.html
  // public static void configureOptimizedPigeon2(Pigeon2 pigeon) {
  //   // 5, 7, 8 don't seem to exist

  //   // calibration status, IMU temperature
  //   pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_1_General, 10000);

  //   // biased gyro values (x, y, z)
  //   pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_2_GeneralCompass, 10000);

  //   // accelerometer derived angles
  //   pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_3_GeneralAccel, 10000);

  //   // unprocessed magnetometer values (x, y, z)
  //   pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.BiasedStatus_4_Mag, 10000);

  //   // 9 degree fused yaw, pitch, roll (requires magnetometer calibration)
  //   pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.BiasedStatus_6_Accel, 10000);

  //   // six degree fused yaw, pitch, roll
  //   pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR, 10);

  //   // six degree fused quaternion
  //   pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_10_SixDeg_Quat, 10000);

  //   // accumulated gyro angles
  //   pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_11_GyroAccum, 10000);
  // }

  public PathPoint pathPointFromHolonomicPose(Pose2d pose) {
    return pathPointFromHolonomicPose(pose, pose.getRotation());
  }

  public PathPoint pathPointFromHolonomicPose(Pose2d pose, Rotation2d pathHeading) {
    return new PathPoint(pose.getTranslation(), pathHeading, pose.getRotation());
  }
}
