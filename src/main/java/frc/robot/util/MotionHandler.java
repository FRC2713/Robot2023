package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Robot;

public class MotionHandler {

  public enum MotionMode {
    FULL_DRIVE,
    SNAPPED_TO_GOAL,
    TRAJECTORY,
    LOCKDOWN
  }

  /**
   * Calculates SwerveModuleState objects using the heading controller.
   *
   * @return The desired array of desaturated swerveModuleStates.
   */
  public static SwerveModuleState[] driveSnappedToGoal() {
    double xSpeed = MathUtil.applyDeadband(-Robot.driver.getLeftY(), DriveConstants.kJoystickTurnDeadzone);
    double ySpeed = MathUtil.applyDeadband(-Robot.driver.getLeftX(), DriveConstants.kJoystickTurnDeadzone);
    SwerveHeadingController.getInstance().setSetpoint(new Rotation2d(Units.degreesToRadians(180)));
    SwerveModuleState[] swerveModuleStates = DriveConstants.kinematics.toSwerveModuleStates(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed * DriveConstants.maxSwerveVel,
            ySpeed * DriveConstants.maxSwerveVel,
            Units.degreesToRadians(SwerveHeadingController.getInstance().update()),
            Robot.swerveDrive.getPose().getRotation()));

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.maxSwerveVel);

    return swerveModuleStates;
  }

  /**
   * Calculates SwerveModuleState objects using pure driver control.
   *
   * @return The desired array of desaturated swerveModuleStates.
   */
  public static SwerveModuleState[] driveFullControl() {
    double xSpeed = MathUtil.applyDeadband(-Robot.driver.getLeftY(), DriveConstants.kJoystickTurnDeadzone);
    double ySpeed = MathUtil.applyDeadband(-Robot.driver.getLeftX(), DriveConstants.kJoystickTurnDeadzone);
    double rSpeed = MathUtil.applyDeadband(-Robot.driver.getRightX(), DriveConstants.kJoystickTurnDeadzone);

    SwerveModuleState[] swerveModuleStates = DriveConstants.kinematics.toSwerveModuleStates(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed * DriveConstants.maxSwerveVel,
            ySpeed * DriveConstants.maxSwerveVel,
            rSpeed * DriveConstants.maxRotationalSpeedRadPerSec,
            Robot.swerveDrive.getPose().getRotation()));

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.maxSwerveVel);

    return swerveModuleStates;
  }

  /**
   * Calculates swerveModuleStates using the current trajectory.
   *
   * @return The desired array of desaturated swerveModuleStates.
   */
  public static SwerveModuleState[] driveTrajectory() {
    SwerveModuleState[] swerveModuleStates = DriveConstants.kinematics
        .toSwerveModuleStates(TrajectoryController.getInstance().update());

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.maxSwerveVel);

    return swerveModuleStates;
  }

  /**
   * Sets the robot to an unmoving lockdown configuration which is difficult to
   * push.
   *
   * @return The lockdown array of swerveModuleStates.
   */
  public static SwerveModuleState[] lockdown() {
    SwerveModuleState[] swerveModuleStates = new SwerveModuleState[] {
        new SwerveModuleState(Constants.zero, Rotation2d.fromDegrees(45)),
        new SwerveModuleState(Constants.zero, Rotation2d.fromDegrees(-45)),
        new SwerveModuleState(Constants.zero, Rotation2d.fromDegrees(-45)),
        new SwerveModuleState(Constants.zero, Rotation2d.fromDegrees(45))
    };

    return swerveModuleStates;
  }
}
