package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.FieldConstants.Community;
import frc.robot.util.MotionHandler.MotionMode;
import frc.robot.util.RedHawkUtil;
import frc.robot.util.TrajectoryController;
import java.util.List;

public class Bridge6328 extends CommandBase {
  public Bridge6328() {}

  @Override
  public void initialize() {
    Pose2d startingPose = Robot.swerveDrive.getUsablePose();
    boolean enterFront =
        startingPose.getX()
            < ((Community.chargingStationInnerX + Community.chargingStationOuterX) / 2.0);

    Pose2d pos0 =
        new Pose2d(
            enterFront ? Community.chargingStationInnerX : Community.chargingStationOuterX,
            MathUtil.clamp(
                startingPose.getY(),
                Community.chargingStationRightY + 0.8,
                Community.chargingStationLeftY - 0.8),
            Rotation2d.fromDegrees(startingPose.getRotation().getCos() > 0 ? 0 : 180));

    Pose2d pos1 =
        new Pose2d(
            (Community.chargingStationOuterX + Community.chargingStationInnerX) / 2.0,
            pos0.getY(),
            pos0.getRotation());

    pos0 = RedHawkUtil.Reflections.reflectIfRed(pos0);
    pos1 = RedHawkUtil.Reflections.reflectIfRed(pos1);

    PathPlannerTrajectory trajectory =
        PathPlanner.generatePath(
            new PathConstraints(
                Constants.DriveConstants.MAX_SWERVE_VEL, Constants.DriveConstants.MAX_SWERVE_ACCEL),
            List.of(
                RedHawkUtil.pathPointFromHolonomicPose(startingPose),
                RedHawkUtil.pathPointFromHolonomicPose(
                    pos0,
                    RedHawkUtil.Reflections.reflectIfRed(
                        Rotation2d.fromDegrees(enterFront ? 0 : 180))),
                RedHawkUtil.pathPointFromHolonomicPose(pos1)));

    TrajectoryController.getInstance().changePath((trajectory));
    Robot.motionMode = MotionMode.TRAJECTORY;
  }

  @Override
  public void execute() {}

  @Override
  public boolean isFinished() {
    return TrajectoryController.getInstance().isFinished();
  }

  @Override
  public void end(boolean interrupted) {
    Robot.motionMode = MotionMode.LOCKDOWN;
  }
}
