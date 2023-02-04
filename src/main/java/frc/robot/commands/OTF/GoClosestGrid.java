package frc.robot.commands.OTF;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Robot;
import frc.robot.util.RedHawkUtil;
import org.littletonrobotics.junction.Logger;

public class GoClosestGrid {
  private PathPlannerTrajectory traj;
  public int count = 0;

  public GoClosestGrid() {
    regenerateTrajectory();
  }

  public PathPlannerTrajectory getTrajectory() {
    return traj;
  }

  public GoClosestGrid regenerateTrajectory() {
    if (RedHawkUtil.getClosestGridNumber(Robot.swerveDrive.getRegularPose().getY()) == 6
        || RedHawkUtil.getClosestGridNumber(Robot.swerveDrive.getRegularPose().getY()) == 5) {
      traj =
          PathPlanner.generatePath(
              new PathConstraints(4, 3),
              new PathPoint(
                  RedHawkUtil.Pose2dToTranslation2d(Robot.swerveDrive.getRegularPose()),
                  Rotation2d.fromDegrees(Robot.swerveDrive.inputs.gyroYawPosition),
                  Robot.swerveDrive.getRegularPose().getRotation()),
              // Top
              new PathPoint(
                  new Translation2d(3.92, 4.62),
                  Rotation2d.fromDegrees(0),
                  Rotation2d.fromDegrees(-180)),
              new PathPoint(
                  RedHawkUtil.getClosestGrid(Robot.swerveDrive.getRegularPose().getY()),
                  Rotation2d.fromDegrees(0),
                  Rotation2d.fromDegrees(-180)));
    } else if (RedHawkUtil.getClosestGridNumber(Robot.swerveDrive.getRegularPose().getY()) == 2
        || RedHawkUtil.getClosestGridNumber(Robot.swerveDrive.getRegularPose().getY()) == 3
        || RedHawkUtil.getClosestGridNumber(Robot.swerveDrive.getRegularPose().getY()) == 4) {
      traj =
          PathPlanner.generatePath(
              new PathConstraints(4, 3),
              new PathPoint(
                  RedHawkUtil.Pose2dToTranslation2d(Robot.swerveDrive.getRegularPose()),
                  Rotation2d.fromDegrees(Robot.swerveDrive.inputs.gyroYawPosition),
                  Robot.swerveDrive.getRegularPose().getRotation()),
              // Bottom
              new PathPoint(
                  new Translation2d(3.92, 0.72),
                  Rotation2d.fromDegrees(0),
                  Rotation2d.fromDegrees(-180)),
              new PathPoint(
                  RedHawkUtil.getClosestGrid(Robot.swerveDrive.getRegularPose().getY()),
                  Rotation2d.fromDegrees(0),
                  Rotation2d.fromDegrees(-180)));
    } else {
      traj =
          PathPlanner.generatePath(
              new PathConstraints(4, 3),
              new PathPoint(
                  RedHawkUtil.Pose2dToTranslation2d(Robot.swerveDrive.getRegularPose()),
                  Rotation2d.fromDegrees(Robot.swerveDrive.inputs.gyroYawPosition),
                  Robot.swerveDrive.getRegularPose().getRotation(),
                  Robot.swerveDrive.getAverageVelocity()),
              //   PathPoint.fromCurrentHolonomicState(
              //       Robot.swerveDrive.getRegularPose(),
              // TrajectoryController.getInstance().update()),
              // position, heading(direction of travel), holonomic rotation, velocity

              new PathPoint(
                  RedHawkUtil.getClosestGrid(Robot.swerveDrive.getRegularPose().getY()),
                  Rotation2d.fromDegrees(0),
                  Rotation2d.fromDegrees(-180),
                  2) // position, heading(direction of travel), holonomic rotation
              );
    }
    count++;
    Logger.getInstance()
        .recordOutput(
            "ClosestGrid Numero",
            RedHawkUtil.getClosestGridNumber(Robot.swerveDrive.getRegularPose().getY()));
    Logger.getInstance().recordOutput("Count Number", count);
    return this;
  }
}
