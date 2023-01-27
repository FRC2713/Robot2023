package frc.robot.util;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.util.Units;

public class AutoPath {
  public static final double fieldLength = Units.inchesToMeters(651.25);
  public static final double fieldWidth = Units.inchesToMeters(315.5);

  public enum Autos {
    GO_TO_FIRST_CARGO("goto1stcargo"),
    GO_TO_GRID("gotogrid"),
    GO_TO_SECOND_CARGO("goto2ndcargo"),
    GO_TO_GRID_TWO("gotogrid2"),
    DOCK("dock"),
    ONE_TO_A("grid1tocargoA"),
    A_TO_THREE("cargoAtogrid3"),
    THREE_TO_BRIDGE("getonthebridge");

    private PathPlannerTrajectory trajectory;

    private Autos(String filename) {
      try {
        this.trajectory =
            ReflectedTransform.reflectiveTransformTrajectory(
                PathPlanner.loadPath(filename, PathPlanner.getConstraintsFromPath(filename)));
      } catch (NullPointerException ex) {
        this.trajectory =
            ReflectedTransform.reflectiveTransformTrajectory(
                PathPlanner.loadPath(filename, new PathConstraints(4, 3)));
      }
      ;
    }

    public PathPlannerTrajectory getTrajectory() {
      return trajectory;
    }
  }
}
