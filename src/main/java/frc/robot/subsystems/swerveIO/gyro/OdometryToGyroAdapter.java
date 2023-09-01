package frc.robot.subsystems.swerveIO.gyro;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.subsystems.swerveIO.module.SwerveModule;

public class OdometryToGyroAdapter {

  private SwerveDriveOdometry odometry;
  private SwerveModule[] modules;

  public OdometryToGyroAdapter() {}

  public void getYPR(double[] yprDegrees) {
        if (modules == null || odometry == null) return;

    SwerveModuleState[] measuredStates =
        new SwerveModuleState[] {
          modules[0].getMeasuredState(),
          modules[1].getMeasuredState(),
          modules[2].getMeasuredState(),
          modules[3].getMeasuredState()
        };

    ChassisSpeeds speeds = Constants.DriveConstants.KINEMATICS.toChassisSpeeds(measuredStates);
    Pose2d odometryPose =
        odometry
            .getPoseMeters()
            .exp(
                new Twist2d(
                    speeds.vxMetersPerSecond * .02,
                    speeds.vyMetersPerSecond * .02,
                    speeds.omegaRadiansPerSecond * .02));

    // Logger.getInstance().recordOutput("Swerve/Sim Adapter/Measured Module States", measuredStates);
    // Logger.getInstance().recordOutput("Swerve/Sim Adapter/Odometry Pose", odometryPose);

    yprDegrees[0] = odometryPose.getRotation().getDegrees();
    yprDegrees[1] = 0.0;
    yprDegrees[2] = 0.0;
  }

  public void setSources(SwerveDriveOdometry odometry, SwerveModule[] modules) {
    this.modules = modules;
    this.odometry = odometry;
  }
}
