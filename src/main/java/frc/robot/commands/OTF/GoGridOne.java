package frc.robot.commands.OTF;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Robot;
import frc.robot.util.RedHawkUtil;

public class GoGridOne {
  // Simple path with holonomic rotation. Stationary start/end. Max velocity of 4 m/s and max accel
  // of 3 m/s^2
  private PathPlannerTrajectory traj;

  public GoGridOne() {
    traj =
        PathPlanner.generatePath(
            new PathConstraints(4, 3),
            new PathPoint(
                RedHawkUtil.Pose2dToTranslation2d(Robot.swerveDrive.getRegularPose()),
                Rotation2d.fromDegrees(Robot.swerveDrive.inputs.gyroYawPosition),
                Robot.swerveDrive.getRegularPose().getRotation()),
            // position, heading(direction of travel), holonomic rotation
            new PathPoint(
                new Translation2d(1.81, 5.04),
                Rotation2d.fromDegrees(0),
                Rotation2d.fromDegrees(
                    -180)) // position, heading(direction of travel), holonomic rotation
            );
  }

  public PathPlannerTrajectory getTrajectory() {
    return traj;
  }
}
