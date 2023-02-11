package frc.robot.commands.OTF.boundingBoxes;

import edu.wpi.first.math.geometry.Pose2d;

public interface BoundingBox {
  public Pose2d[] getPoints();

  public BoundingBox fromArrayofPose2d(Pose2d[] poses);
}
