package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Robot;
import frc.robot.subsystems.swerveIO.SwerveSubsystem;

public class MotionHandler {

  public enum MotionMode {
    FULL_DRIVE,
    HEADING_CONTROLLER,
    TRAJECTORY,
    LOCKDOWN,
    NULL
  }

  /**
   * Calculates SwerveModuleState objects using the heading controller.
   *
   * @return The desired array of desaturated swerveModuleStates.
   */
  public static ChassisSpeeds driveHeadingController() {
    double speedFactor = (Robot.driver.getLeftTriggerAxis() > 0.25) ? 0.33 : 1.0;

    double xSpeed =
        MathUtil.applyDeadband(-Robot.driver.getLeftY(), DriveConstants.K_JOYSTICK_TURN_DEADZONE)
            * speedFactor;
    double ySpeed =
        MathUtil.applyDeadband(-Robot.driver.getLeftX(), DriveConstants.K_JOYSTICK_TURN_DEADZONE)
            * speedFactor;

    ChassisSpeeds chassisSpeedState =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed * DriveConstants.MAX_SWERVE_VEL * SwerveSubsystem.allianceFlipper,
            ySpeed * DriveConstants.MAX_SWERVE_VEL * SwerveSubsystem.allianceFlipper,
            Units.degreesToRadians(SwerveHeadingController.getInstance().update()) * speedFactor,
            Robot.swerveDrive.getYaw());

    return chassisSpeedState;
  }

  /**
   * Calculates SwerveModuleState objects using pure driver control.
   *
   * @return The desired array of desaturated swerveModuleStates.
   */
  public static ChassisSpeeds driveFullControl() {
    double speedFactor = (1);

    double xSpeed =
        MathUtil.applyDeadband(-Robot.driver.getLeftY(), DriveConstants.K_JOYSTICK_TURN_DEADZONE)
            * speedFactor;
    double ySpeed =
        MathUtil.applyDeadband(-Robot.driver.getLeftX(), DriveConstants.K_JOYSTICK_TURN_DEADZONE)
            * speedFactor;
    double rSpeed =
        (-Robot.driver.getLeftTriggerAxis() + Robot.driver.getRightTriggerAxis()) * speedFactor;

    ChassisSpeeds chassisSpeedState =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed * DriveConstants.MAX_SWERVE_VEL * SwerveSubsystem.allianceFlipper,
            ySpeed * DriveConstants.MAX_SWERVE_VEL * SwerveSubsystem.allianceFlipper,
            rSpeed * DriveConstants.MAX_ROTATIONAL_SPEED_RAD_PER_SEC,
            Robot.swerveDrive.getYaw());

    return chassisSpeedState;
  }

  /**
   * Calculates swerveModuleStates using the current trajectory.
   *
   * @return The desired array of desaturated swerveModuleStates.
   */
  public static ChassisSpeeds driveTrajectory() {
    ChassisSpeeds chassisSpeedsState = TrajectoryController.getInstance().update();
    return chassisSpeedsState;
  }

  /**
   * Sets the robot to an unmoving lockdown configuration which is difficult to push.
   *
   * @return The lockdown array of swerveModuleStates.
   */
  public static SwerveModuleState[] lockdown() {
    SwerveModuleState[] swerveModuleStates =
        new SwerveModuleState[] {
          new SwerveModuleState(Constants.zero, Rotation2d.fromDegrees(45)),
          new SwerveModuleState(Constants.zero, Rotation2d.fromDegrees(-45)),
          new SwerveModuleState(Constants.zero, Rotation2d.fromDegrees(-45)),
          new SwerveModuleState(Constants.zero, Rotation2d.fromDegrees(45))
        };

    return swerveModuleStates;
  }
}
