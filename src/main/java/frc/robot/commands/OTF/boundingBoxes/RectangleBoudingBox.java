package frc.robot.commands.OTF.boundingBoxes;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.Objects;
import lombok.NonNull;

public class RectangleBoudingBox implements BoundingBox {
  private final Pose2d[] poses = {null, null};

  public RectangleBoudingBox(Pose2d topLeft, Pose2d bottomRight) {
    poses[0] = topLeft;
    poses[1] = bottomRight;
  }

  @Override
  public RectangleBoudingBox fromArrayofPose2d(@NonNull Pose2d[] poses) {
    if (Objects.isNull(poses[1])) {
      throw new IllegalArgumentException("2 poses must be defined for a rectangle");
    }
    return new RectangleBoudingBox(poses[0], poses[1]);
  }

  @Override
  public Pose2d[] getPoints() {
    return poses;
  }
}
