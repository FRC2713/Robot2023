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

public class GoHumanPlayer {
  private PathPlannerTrajectory traj;
  private ArrayList<PathPoint> points = new ArrayList<>();
  private ArrayList<Pair<Translation2d, PathPoint>> breakPoints = new ArrayList<>();
  private Rotation2d heading = RedHawkUtil.Reflections.reflectIfRed(Rotation2d.fromDegrees(0));
  private final PathConstraints constraints =
      new PathConstraints(
          Constants.DriveConstants.MAX_SWERVE_VEL, Constants.DriveConstants.MAX_SWERVE_ACCEL);

  private final Timer timer;
  private PathPoint station;

  public GoHumanPlayer() {
    timer = new Timer();
    timer.start();
    regenerateTrajectory();
  }

  public GoHumanPlayer regenerateTrajectory() {
    heading = RedHawkUtil.Reflections.reflectIfRed(Rotation2d.fromDegrees(0));

    breakPoints = new ArrayList<>();
    breakPoints.add(
        new Pair<>(
            new Translation2d(
                FieldConstants.Community.chargingStationOuterX,
                FieldConstants.Community.chargingStationLeftY),
            new PathPoint(
                RedHawkUtil.Reflections.reflectIfRed(
                    FieldConstants.Community.Foreign.chargingStationCorners[3].plus(
                        new Translation2d(
                            0, Constants.DriveConstants.FieldTunables.CHARGE_STATION_OFFSET))),
                heading,
                Rotation2d.fromDegrees(90))));

    breakPoints.add(
        new Pair<>(
            new Translation2d(
                FieldConstants.LoadingZone.Foreign.regionCorners[0].getX(),
                FieldConstants.LoadingZone.Foreign.regionCorners[0].getY()),
            new PathPoint(
                RedHawkUtil.Reflections.reflectIfRed(
                    FieldConstants.LoadingZone.regionCorners[0].plus(
                        new Translation2d(
                            0, Constants.DriveConstants.FieldTunables.CHARGE_STATION_OFFSET))),
                heading,
                Rotation2d.fromDegrees(90))));

    station =
        new PathPoint(
            RedHawkUtil.Reflections.reflectIfRed(
                new Translation2d(
                    FieldConstants.LoadingZone.singleSubstationCenterX,
                    FieldConstants.LoadingZone.leftY
                        - Constants.DriveConstants.FieldTunables.SINGLE_HUMAN_STATION_OFFSET)),
            heading,
            Rotation2d.fromDegrees(90));

    points = new ArrayList<>();

    points.add(RedHawkUtil.currentPositionPathPoint(heading));

    for (int i = 0; i < breakPoints.size(); i++) {
      if (((RedHawkUtil.Reflections.reflectIfRed(Robot.swerveDrive.getUsablePose().getX())
              > breakPoints.get(i).getFirst().getX()))
          && (Robot.swerveDrive.getUsablePose().getY() < breakPoints.get(i).getFirst().getY())) {
        points.add(breakPoints.get(i).getSecond());
      }
    }

    points.add(station);

    traj = PathPlanner.generatePath(constraints, points);

    return this;
  }

  public PathPlannerTrajectory getTrajectory() {
    return traj;
  }

  public boolean hasElapsed() {
    Logger.getInstance().recordOutput("OTF/Timer", timer.get());
    Logger.getInstance()
        .recordOutput(
            "OTF/Closest Grid",
            RedHawkUtil.getClosestGridNumber(Robot.swerveDrive.getUsablePose().getY()));
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
}
