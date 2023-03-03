package frc.robot.subsystems.swerveIO;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Robot;
import frc.robot.subsystems.swerveIO.module.SwerveModule;
import frc.robot.subsystems.swerveIO.module.SwerveModuleIO;
import frc.robot.util.MotionHandler;
import frc.robot.util.TrajectoryController;
import org.littletonrobotics.junction.Logger;

public class SwerveSubsystem extends SubsystemBase {

  SwerveIO io;
  public final SwerveInputsAutoLogged inputs = new SwerveInputsAutoLogged();

  private final SwerveModule frontLeft;
  private final SwerveModule frontRight;
  private final SwerveModule backLeft;
  private final SwerveModule backRight;

  private final SwerveDriveOdometry odometry;
  private final SwerveDrivePoseEstimator poseEstimator;
  private Pose2d simOdometryPose;

  private LinearFilter filteredRoll = LinearFilter.singlePoleIIR(0.08, 0.02);
  public double filteredRollVal = 0;

  /**
   * Creates a new SwerveSubsystem (swerve drive) object.
   *
   * @param swerveIO The IO layer of the swerve drive. Change this to change which gyro you're using
   *     (SwerveModuleIOPigeon2 vs SwerveModuleIOSim)
   * @param frontLeft The IO layer for the front left swerve module. Change this to change which
   *     motor controller you're using (SwerveModuleIOSim vs SwerveModuleIOSparkMAX)
   * @param frontRight The IO layer for the front right swerve module.
   * @param backLeft The IO layer for the back left swerve module.
   * @param backRight The IO layer for the back left swerve module.
   */
  public SwerveSubsystem(
      SwerveIO swerveIO,
      SwerveModuleIO frontLeft,
      SwerveModuleIO frontRight,
      SwerveModuleIO backLeft,
      SwerveModuleIO backRight) {
    this.frontLeft = new SwerveModule(frontLeft, Constants.DriveConstants.FRONT_LEFT);
    this.frontRight = new SwerveModule(frontRight, Constants.DriveConstants.FRONT_RIGHT);
    this.backLeft = new SwerveModule(backLeft, Constants.DriveConstants.BACK_LEFT);
    this.backRight = new SwerveModule(backRight, Constants.DriveConstants.BACK_RIGHT);
    io = swerveIO;
    io.updateInputs(inputs);

    odometry =
        new SwerveDriveOdometry(
            DriveConstants.KINEMATICS,
            Rotation2d.fromDegrees(inputs.gyroYawPosition),
            new SwerveModulePosition[] {
              this.frontLeft.getPosition(),
              this.frontRight.getPosition(),
              this.backLeft.getPosition(),
              this.backRight.getPosition()
            },
            new Pose2d());

    poseEstimator =
        new SwerveDrivePoseEstimator(
            DriveConstants.KINEMATICS,
            Rotation2d.fromDegrees(inputs.gyroYawPosition),
            new SwerveModulePosition[] {
              this.frontLeft.getPosition(),
              this.frontRight.getPosition(),
              this.backLeft.getPosition(),
              this.backRight.getPosition()
            },
            new Pose2d(),
            new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.1, 0.1, 0.1),
            new MatBuilder<>(Nat.N3(), Nat.N1())
                .fill(
                    Constants.LimeLightConstants.VISION_STD_DEVI_POSITION_IN_METERS,
                    Constants.LimeLightConstants.VISION_STD_DEVI_POSITION_IN_METERS,
                    Constants.LimeLightConstants.VISION_STD_DEVI_ROTATION_IN_RADIANS));

    simOdometryPose = odometry.getPoseMeters();
  }

  public void zeroGyro() {
    io.zeroGyro();
  }

  /**
   * Sets the gyro to the given rotation.
   *
   * @param rotation The rotation to reset the gyro to.
   */
  public void resetGyro(Rotation2d rotation) {
    io.resetGyro(rotation);
  }

  /**
   * Resets the SwerveDriveOdometry to the given pose.
   *
   * @param pose The desired pose.
   */
  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(
        getUsablePose().getRotation(),
        new SwerveModulePosition[] {
          frontLeft.getPosition(),
          frontRight.getPosition(),
          backLeft.getPosition(),
          backRight.getPosition()
        },
        pose);

    poseEstimator.resetPosition(
        getUsablePose().getRotation(),
        new SwerveModulePosition[] {
          this.frontLeft.getPosition(),
          this.frontRight.getPosition(),
          this.backLeft.getPosition(),
          this.backRight.getPosition()
        },
        pose);
    simOdometryPose = pose;
  }

  /**
   * Returns the current pose of the robot.
   *
   * @return The position of the robot on the field.
   */
  private Pose2d getEstimatedPose() {
    if (Robot.isReal()) {
      return poseEstimator.getEstimatedPosition();
    } else {
      return simOdometryPose;
    }
  }

  public Pose2d getUsablePose() {
    if (Constants.ENABLE_VISION_POSE_ESTIMATION) {
      return getEstimatedPose();
    } else {
      return getRegularPose();
    }
  }

  private Pose2d getRegularPose() {
    if (Robot.isReal()) {
      return odometry.getPoseMeters();
    } else {
      return simOdometryPose;
    }
  }

  public double getTotalCurrentDraw() {
    return frontLeft.getTotalCurrentDraw()
        + frontRight.getTotalCurrentDraw()
        + backLeft.getTotalCurrentDraw()
        + backRight.getTotalCurrentDraw();
  }

  public void updateVisionPose(
      TimestampedDoubleArray fieldPoseArray, TimestampedDoubleArray cameraPoseArray) {
    double[] fVal = fieldPoseArray.value;
    double[] cVal = cameraPoseArray.value;
    double distCamToTag = Units.metersToInches(Math.abs(cVal[2]));

    Logger.getInstance().recordOutput("Vision/distCamToTag", distCamToTag);
    Pose2d fPose = new Pose2d(fVal[0], fVal[1], new Rotation2d(fVal[5]));

    if (fPose.getX() == 0 && fPose.getY() == 0 && fPose.getRotation().getDegrees() == 0) {
      return;
    }

    double jump_distance =
        Units.metersToInches(
            poseEstimator
                .getEstimatedPosition()
                .getTranslation()
                .getDistance(fPose.getTranslation()));
    Logger.getInstance().recordOutput("Vision/jump_distance", jump_distance);
    if (distCamToTag < Constants.LimeLightConstants.CAMERA_TO_TAG_MAX_DIST_INCHES
        && jump_distance < Constants.LimeLightConstants.MAX_POSE_JUMP_IN_INCHES) {
      poseEstimator.addVisionMeasurement(fPose, edu.wpi.first.wpilibj.Timer.getFPGATimestamp());
    }
  }

  /**
   * Sets the desired states of the swerve modules.
   *
   * @param swerveModuleStates The array of desired swerveModuleStates. Ensure they are ordered the
   *     same way in this array as they are instantiated into SwerveDriveKinematics.
   */
  public void setModuleStates(SwerveModuleState[] swerveModuleStates) {
    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    backLeft.setDesiredState(swerveModuleStates[2]);
    backRight.setDesiredState(swerveModuleStates[3]);
  }

  public boolean gyroPitchHasChanged() {
    return inputs.gyroPitchPosition == inputs.previousgyroPitchPosition;
  }

  /**
   * Returns the average velocity of the swerve modules.
   *
   * @return The average velocity at which all the swerve modules are moving.
   */
  public double getAverageVelocity() {
    return (frontLeft.getMeasuredState().speedMetersPerSecond
            + frontRight.getMeasuredState().speedMetersPerSecond
            + backLeft.getMeasuredState().speedMetersPerSecond
            + backRight.getMeasuredState().speedMetersPerSecond)
        / 4;
  }

  // Only used for characterization
  public void applyVoltageForCharacterization(double volts) {
    frontLeft.applyVoltageForCharacterization(volts);
    frontRight.applyVoltageForCharacterization(volts);
    backLeft.applyVoltageForCharacterization(volts);
    backRight.applyVoltageForCharacterization(volts);
  }

  /**
   * Updates the odometry of the robot using the swerve module states and the gyro reading. Should
   * be run in periodic() or during every code loop to maintain accuracy.
   */
  public void updateOdometry() {

    odometry.update(
        Rotation2d.fromDegrees(inputs.gyroYawPosition),
        new SwerveModulePosition[] {
          frontLeft.getPosition(),
          frontRight.getPosition(),
          backLeft.getPosition(),
          backRight.getPosition()
        });

    poseEstimator.updateWithTime(
        Timer.getFPGATimestamp(),
        Rotation2d.fromDegrees(inputs.gyroYawPosition),
        new SwerveModulePosition[] {
          frontLeft.getPosition(),
          frontRight.getPosition(),
          backLeft.getPosition(),
          backRight.getPosition()
        });

    if (Robot.isSimulation()) {
      SwerveModuleState[] measuredStates =
          new SwerveModuleState[] {
            frontLeft.getMeasuredState(),
            frontRight.getMeasuredState(),
            backLeft.getMeasuredState(),
            backRight.getMeasuredState()
          };
      ChassisSpeeds speeds = Constants.DriveConstants.KINEMATICS.toChassisSpeeds(measuredStates);
      simOdometryPose =
          simOdometryPose.exp(
              new Twist2d(
                  speeds.vxMetersPerSecond * .02,
                  speeds.vyMetersPerSecond * .02,
                  speeds.omegaRadiansPerSecond * .02));

      inputs.gyroYawPosition = simOdometryPose.getRotation().getDegrees();
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    updateOdometry();
    filteredRollVal = filteredRoll.calculate(inputs.gyroRollPosition);
    Logger.getInstance().recordOutput("Swerve/Filtered roll", filteredRollVal);

    switch (Robot.motionMode) {
      case FULL_DRIVE:
        setModuleStates(MotionHandler.driveFullControl());
        break;
      case HEADING_CONTROLLER:
        setModuleStates(MotionHandler.driveHeadingController());
        break;
      case LOCKDOWN:
        setModuleStates(MotionHandler.lockdown());
        break;
      case TRAJECTORY:
        setModuleStates(MotionHandler.driveTrajectory());
        break;
      default:
        break;
    }

    Logger.getInstance()
        .recordOutput(
            "Swerve/Measured Module States",
            new SwerveModuleState[] {
              frontLeft.getMeasuredState(),
              frontRight.getMeasuredState(),
              backLeft.getMeasuredState(),
              backRight.getMeasuredState()
            });
    Logger.getInstance()
        .recordOutput(
            "Swerve/Desired Module States",
            new SwerveModuleState[] {
              frontLeft.getDesiredState(),
              frontRight.getDesiredState(),
              backLeft.getDesiredState(),
              backRight.getDesiredState()
            });

    Logger.getInstance().processInputs("Swerve/Chassis", inputs);
    Logger.getInstance()
        .recordOutput(
            "Swerve/Odometry Pose",
            new double[] {
              getRegularPose().getX(),
              getRegularPose().getY(),
              getRegularPose().getRotation().getDegrees()
            });
    Logger.getInstance()
        .recordOutput(
            "Swerve/PoseEstimator Pose",
            new double[] {
              getEstimatedPose().getX(),
              getEstimatedPose().getY(),
              getEstimatedPose().getRotation().getDegrees()
            });
    Logger.getInstance().recordOutput("Swerve/MotionMode", Robot.motionMode.name());
  }

  public static class Commands {
    public static SequentialCommandGroup followTAndWait(PathPlannerTrajectory T) {
      return new SequentialCommandGroup(
          new InstantCommand(() -> TrajectoryController.getInstance().changePath(T)),
          new WaitUntilCommand(() -> TrajectoryController.getInstance().isFinished()));
    }

    public static SequentialCommandGroup stringTrajectoriesTogether(
        PathPlannerTrajectory... trajectories) {
      SequentialCommandGroup masterTrajectory =
          new SequentialCommandGroup(
              new InstantCommand(
                  () -> TrajectoryController.getInstance().changePath(trajectories[0])));

      for (PathPlannerTrajectory t : trajectories) {
        masterTrajectory.addCommands(followTAndWait(t));
      }
      return masterTrajectory;
    }
  }
}
