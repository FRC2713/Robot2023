package frc.robot.util;

import com.revrobotics.REVLibError;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.Robot;
import java.util.ArrayList;
import java.util.Arrays;
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
  public static void errorHandleSparkMAX(@NonNull REVLibError status, @NonNull String name) {
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

  public static Translation2d Pose2dToTranslation2d(Pose2d pose) {
    return new Translation2d(pose.getX(), pose.getY());
  }

  public static Translation2d getClosestGrid(double y) {
    return Arrays.asList(FieldConstants.Grids.complexLowTranslations).stream()
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
            Arrays.asList(FieldConstants.Grids.complexLowTranslations).stream()
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
        return new Translation2d(FieldConstants.fieldLength - old.getX(), old.getY());
      }
      return old;
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
  }
}
