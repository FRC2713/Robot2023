package frc.robot.commands.OTF;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.util.FieldConstants;
import frc.robot.util.RedHawkUtil;
import org.littletonrobotics.junction.Logger;

public class GoClosestGrid {
  private PathPlannerTrajectory traj;
  private final Timer timer;

  public GoClosestGrid() {
    timer = new Timer();
    timer.start();
    regenerateTrajectory();
  }

  public PathPlannerTrajectory getTrajectory() {
    return traj;
  }

  public GoClosestGrid regenerateTrajectory() {
    // Outside community
    if (Robot.swerveDrive.getRegularPose().getX()
        > FieldConstants.Community.chargingStationOuterX) {
      // Top of charge station
      if (RedHawkUtil.getClosestGridNumber(Robot.swerveDrive.getRegularPose().getY()) == 6
          || RedHawkUtil.getClosestGridNumber(Robot.swerveDrive.getRegularPose().getY()) == 5) {
        traj =
            PathPlanner.generatePath(
                new PathConstraints(4, 3),
                new PathPoint(
                    RedHawkUtil.Pose2dToTranslation2d(Robot.swerveDrive.getRegularPose()),
                    Rotation2d.fromDegrees(Robot.swerveDrive.inputs.gyroYawPosition),
                    Robot.swerveDrive.getRegularPose().getRotation()),
                // Top Right
                new PathPoint(
                    FieldConstants.Community.chargingStationCorners[3].plus(
                        new Translation2d(0, 0.5)),
                    Rotation2d.fromDegrees(0),
                    Rotation2d.fromDegrees(-180)),
                // Top Left
                new PathPoint(
                    FieldConstants.Community.chargingStationCorners[1].plus(
                        new Translation2d(0, 0.5)),
                    Rotation2d.fromDegrees(0),
                    Rotation2d.fromDegrees(-180)),
                new PathPoint(
                    RedHawkUtil.getClosestGrid(Robot.swerveDrive.getRegularPose().getY()),
                    Rotation2d.fromDegrees(0),
                    Rotation2d.fromDegrees(-180)));
      }
      // Bottom of charge station
      else if (RedHawkUtil.getClosestGridNumber(Robot.swerveDrive.getRegularPose().getY()) == 2
          || RedHawkUtil.getClosestGridNumber(Robot.swerveDrive.getRegularPose().getY()) == 3
          || RedHawkUtil.getClosestGridNumber(Robot.swerveDrive.getRegularPose().getY()) == 4) {
        traj =
            PathPlanner.generatePath(
                new PathConstraints(4, 3),
                new PathPoint(
                    RedHawkUtil.Pose2dToTranslation2d(Robot.swerveDrive.getRegularPose()),
                    Rotation2d.fromDegrees(Robot.swerveDrive.inputs.gyroYawPosition),
                    Robot.swerveDrive.getRegularPose().getRotation()),
                // Bottom Right
                new PathPoint(
                    FieldConstants.Community.chargingStationCorners[2].minus(
                        new Translation2d(0, 0.5)),
                    Rotation2d.fromDegrees(0),
                    Rotation2d.fromDegrees(-180)),
                // Bottom Left
                new PathPoint(
                    FieldConstants.Community.chargingStationCorners[0].minus(
                        new Translation2d(0, 0.5)),
                    Rotation2d.fromDegrees(0),
                    Rotation2d.fromDegrees(-180)),
                new PathPoint(
                    RedHawkUtil.getClosestGrid(Robot.swerveDrive.getRegularPose().getY()),
                    Rotation2d.fromDegrees(0),
                    Rotation2d.fromDegrees(-180)));
      } else {
        setDefaultTraj();
      }
    } else {
      setDefaultTraj();
    }

    Logger.getInstance()
        .recordOutput(
            "ClosestGrid Numero",
            RedHawkUtil.getClosestGridNumber(Robot.swerveDrive.getRegularPose().getY()));
    return this;
  }

  public boolean hasElapsed() {
    if (timer.hasElapsed(3)) {
      regenerateTrajectory();
      timer.reset();
      timer.start();
      return true;
    }
    return false;
  }

  private void setDefaultTraj() {
    traj =
        PathPlanner.generatePath(
            new PathConstraints(4, 3),
            new PathPoint(
                RedHawkUtil.Pose2dToTranslation2d(Robot.swerveDrive.getRegularPose()),
                // Rotation2d.fromDegrees(180),
                Rotation2d.fromDegrees(Robot.swerveDrive.inputs.gyroYawPosition),
                Robot.swerveDrive.getRegularPose().getRotation(),
                Robot.swerveDrive.getAverageVelocity()),
            // PathPoint.fromCurrentHolonomicState(
            // Robot.swerveDrive.getRegularPose(),
            // TrajectoryController.getInstance().update()),
            // position, heading(direction of travel), holonomic rotation, velocity

            new PathPoint(
                RedHawkUtil.getClosestGrid(Robot.swerveDrive.getRegularPose().getY()),
                Rotation2d.fromDegrees(0),
                Rotation2d.fromDegrees(-180),
                2) // position, heading(direction of travel), holonomic
            // rotation
            );
  }
}
