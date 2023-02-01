package frc.robot.commands.OTF;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Robot;
import frc.robot.util.RedHawkUtil;

public class GoClosestGrid {
  private final PathPlannerTrajectory traj;

  public GoClosestGrid() {
    traj =
        PathPlanner.generatePath(
            new PathConstraints(4, 3),
            new PathPoint(
                RedHawkUtil.Pose2dToTranslation2d(Robot.swerveDrive.getRegularPose()),
                Rotation2d.fromDegrees(Robot.swerveDrive.inputs.gyroYawPosition),
                Robot.swerveDrive.getRegularPose().getRotation()),
            // position, heading(direction of travel), holonomic rotation

            new PathPoint(
                RedHawkUtil.getClosestGrid(Robot.swerveDrive.getRegularPose().getY()),
                Rotation2d.fromDegrees(0),
                Rotation2d.fromDegrees(
                    -180)) // position, heading(direction of travel), holonomic rotation
            );
  }

  public PathPlannerTrajectory getTrajectory() {
    return traj;
  }
}
