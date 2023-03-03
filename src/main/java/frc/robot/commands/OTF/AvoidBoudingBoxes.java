package frc.robot.commands.OTF;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Robot;
import frc.robot.commands.OTF.boundingBoxes.BoundingBox;
import frc.robot.commands.OTF.boundingBoxes.RectangleBoudingBox;
import frc.robot.util.RedHawkUtil;
import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;

public class AvoidBoudingBoxes {
  private final BoundingBox object;

  public AvoidBoudingBoxes(RectangleBoudingBox object) {
    this.object = object;
  }

  // public boolean insideBox(Pose2d currentPose) {
  // // return insideBoxWithPadding(
  // // currentPose, new Pose2d(new Translation2d(0, 0),
  // Rotation2d.fromDegrees(0)));
  // return insideBoxWithPadding(currentPose, 1);
  // }

  private boolean inTop(double realX, double boxX, double realY, double boxY) {
    return (realX > boxX && realY < boxY);
  }

  private boolean inBottom(double realX, double boxX, double realY, double boxY) {
    return (realX < boxX && realY > boxY);
  }

  private static class Comparors {
    private static boolean pastTopLeft(double realX, double boxX, double realY, double boxY) {
      return (realX < boxX && realY > boxY);
    }

    private static boolean pastTopRight(double realX, double boxX, double realY, double boxY) {
      return (realX > boxX && realY > boxY);
    }

    private static boolean pastBottomRight(double realX, double boxX, double realY, double boxY) {
      return (realX > boxX && realY < boxY);
    }

    private static boolean pastBottomLeft(double realX, double boxX, double realY, double boxY) {
      return (realX < boxX && realY < boxY);
    }
  }

  public boolean insideBoxWithPadding(Translation2d currentPose, Translation2d padding) {
    double x = currentPose.getX();
    double y = currentPose.getY();
    Pose2d[] points = object.getPoints();
    Logger.getInstance().recordOutput("TOP LEFT X OLD", points[0].getX());
    Logger.getInstance().recordOutput("TOP LEFT Y OLD", points[0].getY());

    Pose2d top =
        new Pose2d(
            new Translation2d(points[0].getX() - padding.getX(), points[0].getY() + padding.getY()),
            Rotation2d.fromDegrees(0));

    Pose2d bottom =
        new Pose2d(
            new Translation2d(points[1].getX() + padding.getX(), points[1].getY() - padding.getY()),
            Rotation2d.fromDegrees(0));

    Logger.getInstance()
        .recordOutput("PAST TOP", Comparors.pastTopLeft(x, points[0].getX(), y, points[0].getY()));
    Logger.getInstance().recordOutput("TOP LEFT X", top.getX());
    Logger.getInstance().recordOutput("TOP LEFT Y", top.getY());
    Logger.getInstance().recordOutput("X", currentPose.getX());
    Logger.getInstance().recordOutput("Y", currentPose.getY());

    return inTop(x, top.getX(), y, top.getY()) && inBottom(x, bottom.getX(), y, bottom.getY());
  }

  public ArrayList<PathPoint> generateTrajectories(Translation2d currentPose, PathPoint goal) {
    ArrayList<PathPoint> list = new ArrayList<>();
    double x = currentPose.getX();
    double y = currentPose.getY();
    Pose2d[] points = object.getPoints();

    if (!insideBoxWithPadding(
        RedHawkUtil.Pose2dToTranslation2d(Robot.swerveDrive.getRegularPose()),
        new Translation2d(1, 1))) {
      return null;
    }

    Logger.getInstance().recordOutput("Top Left X", points[0].getX());
    Logger.getInstance().recordOutput("Top Left Y", points[0].getY());
    Logger.getInstance().recordOutput("X", x);
    Logger.getInstance().recordOutput("Y", y);

    PathPoint current =
        new PathPoint(
            RedHawkUtil.Pose2dToTranslation2d(Robot.swerveDrive.getRegularPose()),
            Rotation2d.fromDegrees(180),
            Robot.swerveDrive.getRegularPose().getRotation(),
            Robot.swerveDrive.getAverageVelocity());

    PathPlannerTrajectory proposed =
        PathPlanner.generatePath(new PathConstraints(3, 4), current, goal);

    ArrayList<Pose2d> states = new ArrayList<>();

    for (double i = 0; i < (proposed.getTotalTimeSeconds()); i++) {
      states.add(proposed.sample(i).poseMeters);
    }

    Logger.getInstance().recordOutput("PROPOSED PATH TIME SECONDS", proposed.getTotalTimeSeconds());

    Logger.getInstance().recordOutput("STATES LENGTH", states.size());

    for (int i = 0; i < states.size(); i++) {
      Logger.getInstance()
          .recordOutput(
              "INSIDE BOX",
              insideBoxWithPadding(
                  RedHawkUtil.Pose2dToTranslation2d(states.get(i)), new Translation2d(1, 1)));
      if (insideBoxWithPadding(
          RedHawkUtil.Pose2dToTranslation2d(states.get(i)), new Translation2d(1, 1))) {
        Logger.getInstance()
            .recordOutput(
                "PAST BOTTOM RIGHT",
                Comparors.pastBottomRight(
                    states.get(i).getX(),
                    object.getPoints()[1].getX(),
                    states.get(i).getY(),
                    object.getPoints()[1].getY()));

        if (Comparors.pastTopLeft(
            states.get(i).getX(),
            object.getPoints()[0].getX(),
            states.get(i).getY(),
            object.getPoints()[0].getY())) {
          list.add(
              new PathPoint(
                  RedHawkUtil.Pose2dToTranslation2d(object.getPoints()[0]),
                  Rotation2d.fromDegrees(180)));
        }
        if (Comparors.pastTopRight(
            states.get(i).getX(),
            object.getPoints()[1].getX(),
            states.get(i).getY(),
            object.getPoints()[0].getY())) {
          list.add(
              new PathPoint(
                  new Translation2d(object.getPoints()[1].getX(), object.getPoints()[0].getY()),
                  Rotation2d.fromDegrees(180)));
        }
        if (Comparors.pastBottomLeft(
            states.get(i).getX(),
            object.getPoints()[0].getX(),
            states.get(i).getY(),
            object.getPoints()[1].getY())) {
          list.add(
              new PathPoint(
                  new Translation2d(object.getPoints()[0].getX(), object.getPoints()[1].getY()),
                  Rotation2d.fromDegrees(180)));
        }
        if (Comparors.pastBottomRight(
            states.get(i).getX(),
            object.getPoints()[1].getX(),
            states.get(i).getY(),
            object.getPoints()[1].getY())) {
          list.add(
              new PathPoint(
                  RedHawkUtil.Pose2dToTranslation2d(object.getPoints()[1]),
                  Rotation2d.fromDegrees(180)));
        }
      }
    }

    // Logger.getInstance().recordOutput("STATES/STATES 1", states.get(1));
    // Logger.getInstance().recordOutput("STATES/STATES 2", states.get(2));
    // Logger.getInstance().recordOutput("St", null);

    list.add(current);

    // if (Comparors.pastTop(x, points[0].getX(), y, points[0].getY())) {
    // list.add(
    // new PathPoint(
    // new Translation2d(points[0].getX(), points[0].getY()),
    // Rotation2d.fromDegrees(180)));
    // }
    // if (Comparors.pastTop(x, x, y, y))
    list.add(goal);

    return list;
  }

  public ArrayList<PathPoint> generateTrajectories(Translation2d currentPose) {
    Logger.getInstance().recordOutput("GOAL", object.getPoints()[1].getX());

    return generateTrajectories(
        currentPose,
        new PathPoint(
            new Translation2d(object.getPoints()[1].getX(), object.getPoints()[1].getY()),
            Rotation2d.fromDegrees(180)));
  }
}
