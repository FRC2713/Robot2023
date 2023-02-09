package frc.robot.commands.OTF;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.FieldConstants;
import frc.robot.util.RedHawkUtil;
import java.util.ArrayList;

public class GoClosestGrid {
  private PathPlannerTrajectory traj;
  private final Timer timer;
  private PathPoint targetGrid;
  public boolean hasSetTargetGrid = false;
  private ArrayList<PathPoint> points = new ArrayList<>();

  private ArrayList<Pair<Double, PathPoint>> breakPointsTop = new ArrayList<>();
  private ArrayList<Pair<Double, PathPoint>> breakPointsBottom = new ArrayList<>();

  private final Rotation2d heading =
      RedHawkUtil.Reflections.reflectIfRed(Rotation2d.fromDegrees(180));
  private final PathConstraints constraints =
      new PathConstraints(
          Constants.DriveConstants.maxSwerveVel, Constants.DriveConstants.maxSwerveAccel);

  public GoClosestGrid() {
    breakPointsTop.add(
        new Pair<>(
            FieldConstants.Community.chargingStationOuterX,
            new PathPoint(
                FieldConstants.Community.chargingStationCorners[3].plus(
                    RedHawkUtil.Reflections.reflectIfRed(
                        new Translation2d(
                            0, Constants.DriveConstants.FieldTunables.CHARGE_STATION_OFFSET))),
                Constants.DriveConstants.FieldTunables.CLOSEST_GRID_HEADING,
                Constants.DriveConstants.FieldTunables.CLOSEST_GRID_HEADING)));
    breakPointsTop.add(
        new Pair<>(
            FieldConstants.Community.chargingStationInnerX,
            new PathPoint(
                FieldConstants.Community.chargingStationCorners[1].plus(
                    RedHawkUtil.Reflections.reflectIfRed(
                        new Translation2d(
                            0, Constants.DriveConstants.FieldTunables.CHARGE_STATION_OFFSET))),
                Constants.DriveConstants.FieldTunables.CLOSEST_GRID_HEADING,
                Constants.DriveConstants.FieldTunables.CLOSEST_GRID_HEADING)));

    breakPointsBottom.add(
        new Pair<>(
            FieldConstants.Community.chargingStationOuterX,
            new PathPoint(
                FieldConstants.Community.chargingStationCorners[2].minus(
                    new Translation2d(
                        0, Constants.DriveConstants.FieldTunables.CHARGE_STATION_OFFSET)),
                Constants.DriveConstants.FieldTunables.CLOSEST_GRID_HEADING,
                Constants.DriveConstants.FieldTunables.CLOSEST_GRID_HEADING)));
    breakPointsBottom.add(
        new Pair<>(
            FieldConstants.Community.chargingStationInnerX,
            new PathPoint(
                FieldConstants.Community.chargingStationCorners[0].minus(
                    new Translation2d(
                        0, Constants.DriveConstants.FieldTunables.CHARGE_STATION_OFFSET)),
                Constants.DriveConstants.FieldTunables.CLOSEST_GRID_HEADING,
                Constants.DriveConstants.FieldTunables.CLOSEST_GRID_HEADING)));
    timer = new Timer();
    timer.start();
    regenerateTrajectory();
  }

  public PathPlannerTrajectory getTrajectory() {
    return traj;
  }

  public GoClosestGrid regenerateTrajectory() {
    points = new ArrayList<>();

    points.add(currentPosition());

    // Top
    if (RedHawkUtil.getClosestGridNumber(Robot.swerveDrive.getRegularPose().getY())
            <= Constants.DriveConstants.FieldTunables.MAX_GO_TOP
        && RedHawkUtil.getClosestGridNumber(Robot.swerveDrive.getRegularPose().getY())
            >= Constants.DriveConstants.FieldTunables.MIN_GO_TOP) {
      for (int i = 0; i < breakPointsTop.size(); i++) {
        if (Robot.swerveDrive.getRegularPose().getX() > breakPointsTop.get(i).getFirst()) {
          points.add(breakPointsTop.get(i).getSecond());
        }
      }
    }

    // Bottom
    else if (RedHawkUtil.getClosestGridNumber(Robot.swerveDrive.getRegularPose().getY())
            <= Constants.DriveConstants.FieldTunables.MAX_GO_BOTTOM
        && RedHawkUtil.getClosestGridNumber(Robot.swerveDrive.getRegularPose().getY())
            >= Constants.DriveConstants.FieldTunables.MIN_GO_BOTTOM) {
      for (int i = 0; i < breakPointsBottom.size(); i++) {
        if (Robot.swerveDrive.getRegularPose().getX() > breakPointsBottom.get(i).getFirst()) {
          points.add(breakPointsBottom.get(i).getSecond());
        }
      }
    }

    points.add(closestGrid());

    traj = PathPlanner.generatePath(constraints, points);
    return this;
    /*
     *
     * ArrayList<PathPoint> points = new ArrayList<>();
     *
     * final breakpointsUper = [
     * (topRightCornerBridge, new Pose(a, b)),
     * (topLeftCornerBridge, new Pose(c, d)),
     * (someOtherPoint, new Pose(e, f))
     * ]
     * final breakpointsLower = [ ... ]
     *
     * breakpointsToUse = shouldGoUp() ? breakpointsUpper : breakpointsLower;
     * points.add(currentPose())
     * for each breakpoint in breakpoints {
     * if robot to the right of breakpoint.getFirst() {
     * points.add(breakpoint.getSecond())
     * }
     * }
     * points.add(finishingPose)
     *
     * traj = ...
     *
     *
     *
     *
     */
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

  private PathPoint closestGrid() {
    if (!hasSetTargetGrid) {
      PathPoint closest =
          new PathPoint(
              RedHawkUtil.Reflections.reflectIfRed(
                  RedHawkUtil.getClosestGrid(Robot.swerveDrive.getRegularPose().getY())),
              Constants.DriveConstants.FieldTunables.CLOSEST_GRID_HEADING,
              Constants.DriveConstants.FieldTunables.CLOSEST_GRID_HEADING,
              2);
      targetGrid = closest;
      return closest;
    } else {
      return targetGrid;
    }
  }

  private PathPoint currentPosition() {
    return new PathPoint(
        RedHawkUtil.Pose2dToTranslation2d(Robot.swerveDrive.getRegularPose()),
        heading,
        Robot.swerveDrive.getRegularPose().getRotation(),
        Robot.swerveDrive.getAverageVelocity());
  }
}
