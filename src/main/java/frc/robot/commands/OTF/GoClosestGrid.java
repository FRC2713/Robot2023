package frc.robot.commands.OTF;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.FieldConstants;
import frc.robot.util.RedHawkUtil;
import frc.robot.util.Triple;
import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;

public class GoClosestGrid {
  private PathPlannerTrajectory traj;
  private final Timer timer;
  private PathPoint targetGrid;
  public boolean hasSetTargetGrid = false;
  private ArrayList<PathPoint> points = new ArrayList<>();

  private ArrayList<Triple<Double, Double, PathPoint>> breakPointsTop = new ArrayList<>();
  private ArrayList<Triple<Double, Double, PathPoint>> breakPointsBottom = new ArrayList<>();

  private Rotation2d heading = RedHawkUtil.Reflections.reflectIfRed(Rotation2d.fromDegrees(180));
  private final PathConstraints constraints =
      new PathConstraints(
          Constants.DriveConstants.maxSwerveVel, Constants.DriveConstants.maxSwerveAccel);

  public GoClosestGrid() {
    timer = new Timer();
    timer.start();
    regenerateTrajectory();
  }

  public PathPlannerTrajectory getTrajectory() {
    return traj;
  }

  public GoClosestGrid regenerateTrajectory() {
    heading =
        RedHawkUtil.Reflections.reflectIfRed(
            Constants.DriveConstants.FieldTunables.CLOSEST_GRID_HEADING);
    points = new ArrayList<>();

    breakPointsTop = new ArrayList<>();
    breakPointsBottom = new ArrayList<>();

    breakPointsTop.add(
        new Triple<>(
            FieldConstants.Community.chargingStationOuterX,
            -1.0,
            new PathPoint(
                RedHawkUtil.Reflections.reflectIfRed(
                    FieldConstants.Community.chargingStationCorners[3].plus(
                        new Translation2d(
                            0, Constants.DriveConstants.FieldTunables.CHARGE_STATION_OFFSET))),
                heading,
                heading)));
    breakPointsTop.add(
        new Triple<>(
            FieldConstants.Community.chargingStationInnerX,
            -1.0,
            new PathPoint(
                RedHawkUtil.Reflections.reflectIfRed(
                    FieldConstants.Community.chargingStationCorners[1].plus(
                        new Translation2d(
                            0, Constants.DriveConstants.FieldTunables.CHARGE_STATION_OFFSET))),
                heading,
                heading)));
    // TODO: Avoid ridge and foreign starting zone
    // breakPointsTop.add(
    // new Triple<>(
    // -1.0,
    // FieldConstants.Community.leftY,
    // new PathPoint(
    // RedHawkUtil.Reflections.reflectIfRed(
    // FieldConstants.Community.chargingStationCorners[1].plus(
    // new Translation2d(3, 3))),
    // heading,
    // heading)));

    breakPointsBottom.add(
        new Triple<>(
            FieldConstants.Community.chargingStationOuterX,
            -1.0,
            new PathPoint(
                RedHawkUtil.Reflections.reflectIfRed(
                    FieldConstants.Community.chargingStationCorners[2].minus(
                        new Translation2d(
                            0, Constants.DriveConstants.FieldTunables.CHARGE_STATION_OFFSET))),
                heading,
                heading)));

    breakPointsBottom.add(
        new Triple<>(
            FieldConstants.Community.chargingStationInnerX,
            0.0,
            new PathPoint(
                RedHawkUtil.Reflections.reflectIfRed(
                    FieldConstants.Community.chargingStationCorners[0].minus(
                        new Translation2d(
                            0, Constants.DriveConstants.FieldTunables.CHARGE_STATION_OFFSET))),
                heading,
                heading)));

    points.add(currentPosition());

    // Top
    if (RedHawkUtil.getClosestGridNumber(Robot.swerveDrive.getRegularPose().getY())
            <= Constants.DriveConstants.FieldTunables.MAX_GO_TOP
        && RedHawkUtil.getClosestGridNumber(Robot.swerveDrive.getRegularPose().getY())
            >= Constants.DriveConstants.FieldTunables.MIN_GO_TOP) {
      for (int i = 0; i < breakPointsTop.size(); i++) {
        if (inBounds(breakPointsBottom.get(i))) {
          points.add(breakPointsTop.get(i).getThird());
        }
      }
      // Iterator<Triple<Double, Double, PathPoint>> iter = breakPointsTop.iterator();
      // while (iter.hasNext()) {
      // Triple<Double, Double, PathPoint> next = iter.next();
      // if ((Robot.swerveDrive.getRegularPose().getX() > next.getFirst())
      // && (Robot.swerveDrive.getRegularPose().getY() > next.getSecond())) {
      // points.add(next.getThird());
      // }
      // }
    }

    // Bottom
    else if (RedHawkUtil.getClosestGridNumber(Robot.swerveDrive.getRegularPose().getY())
            <= Constants.DriveConstants.FieldTunables.MAX_GO_BOTTOM
        && RedHawkUtil.getClosestGridNumber(Robot.swerveDrive.getRegularPose().getY())
            >= Constants.DriveConstants.FieldTunables.MIN_GO_BOTTOM) {
      for (int i = 0; i < breakPointsBottom.size(); i++) {
        if (inBounds(breakPointsBottom.get(i))) {
          points.add(breakPointsBottom.get(i).getThird());
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
    Logger.getInstance().recordOutput("OTF/Timer", timer.get());
    Logger.getInstance()
        .recordOutput(
            "OTF/Closest Grid",
            RedHawkUtil.getClosestGridNumber(Robot.swerveDrive.getRegularPose().getY()));
    Logger.getInstance()
        .recordOutput(
            "OTF/Is above charge station",
            RedHawkUtil.getClosestGridNumber(Robot.swerveDrive.getRegularPose().getY())
                    <= Constants.DriveConstants.FieldTunables.MAX_GO_TOP
                && RedHawkUtil.getClosestGridNumber(Robot.swerveDrive.getRegularPose().getY())
                    >= Constants.DriveConstants.FieldTunables.MIN_GO_TOP);
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
              heading,
              heading,
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

  private boolean inBounds(Triple<Double, Double, PathPoint> triple) {
    return RedHawkUtil.Reflections.reflectIfRed(Robot.swerveDrive.getRegularPose().getX())
            > triple.getFirst()
        && (Robot.swerveDrive.getRegularPose().getY() > triple.getSecond());
  }
}
