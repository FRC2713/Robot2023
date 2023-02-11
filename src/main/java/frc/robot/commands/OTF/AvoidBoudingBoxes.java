package frc.robot.commands.OTF;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.commands.OTF.boundingBoxes.BoundingBox;
import org.littletonrobotics.junction.Logger;

public class AvoidBoudingBoxes {
  private final BoundingBox object;

  public AvoidBoudingBoxes(BoundingBox object) {
    this.object = object;
  }

  // public boolean insideBox(Pose2d currentPose) {
  // // return insideBoxWithPadding(
  // // currentPose, new Pose2d(new Translation2d(0, 0),
  // Rotation2d.fromDegrees(0)));
  // return insideBoxWithPadding(currentPose, 1);
  // }

  public boolean insideBoxWithPadding(Pose2d currentPose, Translation2d padding) {
    double x = currentPose.getX();
    double y = currentPose.getY();
    Pose2d[] points = object.getPoints();
    Logger.getInstance().recordOutput("TOP LEFT X OLD", points[0].getX());
    Logger.getInstance().recordOutput("TOP LEFT Y OLD", points[0].getY());

    points[0] =
        points[0].plus(
            new Transform2d(
                new Translation2d(padding.getX(), padding.getY()), Rotation2d.fromDegrees(0)));

    Logger.getInstance().recordOutput("TOP LEFT X", points[0].getX());
    Logger.getInstance().recordOutput("TOP LEFT Y", points[0].getY());
    Logger.getInstance().recordOutput("X", currentPose.getX());
    Logger.getInstance().recordOutput("Y", currentPose.getY());

    return (x > points[0].getX() && y < points[0].getY())
        && (x < points[1].getX() && y > points[1].getY());
  }

  public boolean generateTrajectories(Pose2d currentPose) {
    double x = currentPose.getX();
    double y = currentPose.getY();
    Pose2d[] points = object.getPoints();

    Logger.getInstance().recordOutput("Top Left X", points[0].getX());
    Logger.getInstance().recordOutput("Top Left Y", points[0].getY());
    Logger.getInstance().recordOutput("X", x);
    Logger.getInstance().recordOutput("Y", y);

    return false;
  }
}
