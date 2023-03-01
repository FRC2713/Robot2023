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
import org.littletonrobotics.junction.Logger;

public class GoClosestGrid {
  private PathPlannerTrajectory traj;
  private final Timer timer;
  private PathPoint targetGrid;
  private boolean hasSetTargetGrid = false;
  private ArrayList<PathPoint> points = new ArrayList<>();

  private ArrayList<Pair<Double, PathPoint>> breakPointsTop = new ArrayList<>();
  private ArrayList<Pair<Double, PathPoint>> breakPointsBottom = new ArrayList<>();

  private Rotation2d heading = RedHawkUtil.Reflections.reflectIfRed(Rotation2d.fromDegrees(180));
  private final PathConstraints constraints =
      new PathConstraints(
          Constants.DriveConstants.MAX_SWERVE_VEL, Constants.DriveConstants.MAX_SWERVE_ACCEL);

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

    // TODO: Avoid ridge and foreign starting zone
    breakPointsTop.add(
        new Pair<>(
            FieldConstants.Community.chargingStationOuterX,
            new PathPoint(
                RedHawkUtil.Reflections.reflectIfRed(
                    FieldConstants.Community.chargingStationCorners[3].plus(
                        new Translation2d(
                            0, Constants.DriveConstants.FieldTunables.CHARGE_STATION_OFFSET))),
                heading,
                heading)));
    breakPointsTop.add(
        new Pair<>(
            FieldConstants.Community.chargingStationInnerX,
            new PathPoint(
                RedHawkUtil.Reflections.reflectIfRed(
                    FieldConstants.Community.chargingStationCorners[1].plus(
                        new Translation2d(
                            0, Constants.DriveConstants.FieldTunables.CHARGE_STATION_OFFSET))),
                heading,
                heading)));

    breakPointsBottom.add(
        new Pair<>(
            FieldConstants.Community.chargingStationOuterX,
            new PathPoint(
                RedHawkUtil.Reflections.reflectIfRed(
                    FieldConstants.Community.chargingStationCorners[2].minus(
                        new Translation2d(
                            0, Constants.DriveConstants.FieldTunables.CHARGE_STATION_OFFSET))),
                heading,
                heading)));

    breakPointsBottom.add(
        new Pair<>(
            FieldConstants.Community.chargingStationInnerX,
            new PathPoint(
                RedHawkUtil.Reflections.reflectIfRed(
                    FieldConstants.Community.chargingStationCorners[0].minus(
                        new Translation2d(
                            0, Constants.DriveConstants.FieldTunables.CHARGE_STATION_OFFSET))),
                heading,
                heading)));

    points.add(RedHawkUtil.currentPositionPathPoint(heading));

    // Top
    if (RedHawkUtil.getClosestGridNumber(Robot.swerveDrive.getUsablePose().getY())
            <= Constants.DriveConstants.FieldTunables.MAX_GO_TOP
        && RedHawkUtil.getClosestGridNumber(Robot.swerveDrive.getUsablePose().getY())
            >= Constants.DriveConstants.FieldTunables.MIN_GO_TOP) {
      for (int i = 0; i < breakPointsTop.size(); i++) {
        if ((RedHawkUtil.Reflections.reflectIfRed(Robot.swerveDrive.getUsablePose().getX())
            > breakPointsTop.get(i).getFirst())) {
          points.add(breakPointsTop.get(i).getSecond());
        }
      }
    }

    // Bottom
    else if (RedHawkUtil.getClosestGridNumber(Robot.swerveDrive.getUsablePose().getY())
            <= Constants.DriveConstants.FieldTunables.MAX_GO_BOTTOM
        && RedHawkUtil.getClosestGridNumber(Robot.swerveDrive.getUsablePose().getY())
            >= Constants.DriveConstants.FieldTunables.MIN_GO_BOTTOM) {
      for (int i = 0; i < breakPointsBottom.size(); i++) {
        if ((RedHawkUtil.Reflections.reflectIfRed(Robot.swerveDrive.getUsablePose().getX())
            > breakPointsBottom.get(i).getFirst())) {
          points.add(breakPointsBottom.get(i).getSecond());
        }
      }
    }

    points.add(closestGrid());

    traj = PathPlanner.generatePath(constraints, points);
    return this;
  }

  public boolean hasElapsed() {
    Logger.getInstance().recordOutput("OTF/Timer", timer.get());
    Logger.getInstance()
        .recordOutput(
            "OTF/Time to regeneration",
            (Constants.DriveConstants.FieldTunables.TIME_BETWEEN_REGERATION_SECONDS - timer.get()));
    if (timer.hasElapsed(Constants.DriveConstants.FieldTunables.TIME_BETWEEN_REGERATION_SECONDS)) {
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
                  RedHawkUtil.getClosestGrid(Robot.swerveDrive.getUsablePose().getY())),
              heading,
              heading,
              2);
      targetGrid = closest;
      hasSetTargetGrid = true;
      return closest;
    } else {
      return targetGrid;
    }
  }

  public void changingPath() {
    hasSetTargetGrid = false;
  }
}
