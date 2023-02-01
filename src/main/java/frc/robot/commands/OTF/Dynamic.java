package frc.robot.commands.OTF;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.util.RedHawkUtil;

public class Dynamic extends CommandBase {
  private final Pose2d currentPose;
  private final Pose2d mod;

  public Dynamic() {
    currentPose = Robot.swerveDrive.getRegularPose();
    mod = currentPose.plus(new Transform2d(new Translation2d(2, 3), Rotation2d.fromDegrees(45)));
  }

  public PathPlannerTrajectory getTrajectory() {
    return PathPlanner.generatePath(
        new PathConstraints(4, 3),
        new PathPoint(
            RedHawkUtil.Pose2dToTranslation2d(currentPose),
            Rotation2d.fromDegrees(Robot.swerveDrive.inputs.gyroYawPosition),
            currentPose.getRotation()),
        // position, heading(direction of travel), holonomic rotation
        new PathPoint(
            RedHawkUtil.Pose2dToTranslation2d(mod),
            Rotation2d.fromDegrees(45),
            mod.getRotation()) // position, heading(direction of travel), holonomic rotation
        );
  }
}
