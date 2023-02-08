package frc.robot.commands.OTF;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
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

  private ArrayList<Pair<Double, PathPoint>> breakPointsUpper = new ArrayList<>();

  private final Rotation2d heading = Rotation2d.fromDegrees(180);
  private final PathConstraints constraints = new PathConstraints(
      Constants.DriveConstants.maxSwerveVel, Constants.DriveConstants.maxSwerveAccel);

  public GoClosestGrid() {
    breakPointsUpper.add(new Pair<>(FieldConstants.Community.chargingStationOuterX, new PathPoint(
        FieldConstants.Community.chargingStationCorners[3].plus(
            new Translation2d(
                0, Constants.DriveConstants.FieldTunables.CHARGE_STATION_OFFSET)),
        Constants.DriveConstants.FieldTunables.CLOSEST_GRID_HEADING,
        Constants.DriveConstants.FieldTunables.CLOSEST_GRID_HEADING)));
    breakPointsUpper.add(new Pair<>(FieldConstants.Community.chargingStationInnerX, new PathPoint(
      FieldConstants.Community.chargingStationCorners[1].plus(
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

    // Outside community
    if (Robot.swerveDrive.getRegularPose().getX() > FieldConstants.Community.chargingStationOuterX) {
      // Top of charge station
      if (RedHawkUtil.getClosestGridNumber(
          Robot.swerveDrive.getRegularPose().getY()) <= Constants.DriveConstants.FieldTunables.MAX_GO_TOP
          && RedHawkUtil.getClosestGridNumber(
              Robot.swerveDrive.getRegularPose().getY()) >= Constants.DriveConstants.FieldTunables.MIN_GO_TOP) {
        // Current Position
        points.add(currentPosition());

        // Top Right
        points.add(
            new PathPoint(
                FieldConstants.Community.chargingStationCorners[3].plus(
                    new Translation2d(
                        0, Constants.DriveConstants.FieldTunables.CHARGE_STATION_OFFSET)),
                Constants.DriveConstants.FieldTunables.CLOSEST_GRID_HEADING,
                Constants.DriveConstants.FieldTunables.CLOSEST_GRID_HEADING));
        // Top Left
        points.add(
            new PathPoint(
                FieldConstants.Community.chargingStationCorners[1].plus(
                    new Translation2d(
                        0, Constants.DriveConstants.FieldTunables.CHARGE_STATION_OFFSET)),
                Constants.DriveConstants.FieldTunables.CLOSEST_GRID_HEADING,
                Constants.DriveConstants.FieldTunables.CLOSEST_GRID_HEADING));

        // Closest Grid
        points.add(closestGrid());
      }
      // Bottom of charge station
      else if (RedHawkUtil.getClosestGridNumber(
          Robot.swerveDrive.getRegularPose().getY()) <= Constants.DriveConstants.FieldTunables.MAX_GO_BOTTOM
          && RedHawkUtil.getClosestGridNumber(
              Robot.swerveDrive.getRegularPose().getY()) >= Constants.DriveConstants.FieldTunables.MIN_GO_BOTTOM) {
        // Current Position
        points.add(currentPosition());
        // Bottom Right
        points.add(
            new PathPoint(
                FieldConstants.Community.chargingStationCorners[2].minus(
                    new Translation2d(
                        0, Constants.DriveConstants.FieldTunables.CHARGE_STATION_OFFSET)),
                Constants.DriveConstants.FieldTunables.CLOSEST_GRID_HEADING,
                Constants.DriveConstants.FieldTunables.CLOSEST_GRID_HEADING));
        // Bottom Left
        points.add(
            new PathPoint(
                FieldConstants.Community.chargingStationCorners[0].minus(
                    new Translation2d(
                        0, Constants.DriveConstants.FieldTunables.CHARGE_STATION_OFFSET)),
                Constants.DriveConstants.FieldTunables.CLOSEST_GRID_HEADING,
                Constants.DriveConstants.FieldTunables.CLOSEST_GRID_HEADING));

        // Closest Grid
        points.add(closestGrid());
      }
      // Not in front of Charge Station
      else {
        // Current Position
        points.add(currentPosition());

        // Closest Grid
        points.add(closestGrid());
      }
    }
    // Inside community but before begininning of charge station
    else if (Robot.swerveDrive.getRegularPose().getX() > FieldConstants.Community.chargingStationInnerX) {
      // Top of charge station
      if (RedHawkUtil.getClosestGridNumber(
          Robot.swerveDrive.getRegularPose().getY()) <= Constants.DriveConstants.FieldTunables.MAX_GO_TOP
          && RedHawkUtil.getClosestGridNumber(
              Robot.swerveDrive.getRegularPose().getY()) >= Constants.DriveConstants.FieldTunables.MIN_GO_TOP) {
        // Current Position
        points.add(currentPosition());
        // Top Left
        points.add(
            new PathPoint(
                FieldConstants.Community.chargingStationCorners[1].plus(
                    new Translation2d(
                        0, Constants.DriveConstants.FieldTunables.CHARGE_STATION_OFFSET)),
                Constants.DriveConstants.FieldTunables.CLOSEST_GRID_HEADING,
                Constants.DriveConstants.FieldTunables.CLOSEST_GRID_HEADING));

        // Closest Grid
        points.add(closestGrid());
      }
      // Bottom of charge station
      else if (RedHawkUtil.getClosestGridNumber(
          Robot.swerveDrive.getRegularPose().getY()) <= Constants.DriveConstants.FieldTunables.MAX_GO_BOTTOM
          && RedHawkUtil.getClosestGridNumber(
              Robot.swerveDrive.getRegularPose().getY()) >= Constants.DriveConstants.FieldTunables.MIN_GO_BOTTOM) {
        // Current Position
        points.add(currentPosition());
        // Bottom Left
        points.add(
            new PathPoint(
                FieldConstants.Community.chargingStationCorners[0].minus(
                    new Translation2d(
                        0, Constants.DriveConstants.FieldTunables.CHARGE_STATION_OFFSET)),
                Constants.DriveConstants.FieldTunables.CLOSEST_GRID_HEADING,
                Constants.DriveConstants.FieldTunables.CLOSEST_GRID_HEADING));

        // Closest Grid
        points.add(closestGrid());
      }
    } else {
      // Current Position
      points.add(currentPosition());

      // Closest Grid
      points.add(closestGrid());
    }

    traj = PathPlanner.generatePath(constraints, points);
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

  private PathPoint closestGrid() {
    if (!hasSetTargetGrid) {
      PathPoint closest = new PathPoint(
          RedHawkUtil.getClosestGrid(Robot.swerveDrive.getRegularPose().getY()),
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
