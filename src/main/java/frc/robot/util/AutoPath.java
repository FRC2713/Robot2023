package frc.robot.util;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.util.Units;

public class AutoPath {
  public static final double fieldLength = Units.inchesToMeters(651.25);
  public static final double fieldWidth = Units.inchesToMeters(315.5);

  public enum Autos {
    DOCK("dock"),
    A_TO_FIVE("cargoAtogrid5"),
    A_TO_THREE("cargoAtogrid3"),
    D_TO_SEVEN("cargoDtogrid7"),
    ONE_TO_A("grid1tocargoA"),
    THREE_TO_B("grid3tocargoB"),
    NINE_TO_D("grid9tocargoD");

    private PathPlannerTrajectory trajectory;

    private Autos(String filename) {
      try {
        this.trajectory =
            ReflectedTransform.reflectiveTransformTrajectory(
                PathPlanner.loadPath(filename, PathPlanner.getConstraintsFromPath(filename)));
      } catch (NullPointerException ex) {
        try {
          this.trajectory =
              ReflectedTransform.reflectiveTransformTrajectory(
                  PathPlanner.loadPath(filename, new PathConstraints(4, 3)));
        } catch (NullPointerException notAgain) {
          System.out.println(filename + "is not found.");
          RedHawkUtil.ErrHandler.getInstance()
              .addError(filename + "Is Not Found. This is really bad guys this file is not found");
        }
      }
    }

    public PathPlannerTrajectory getTrajectory() {
      return trajectory;
    }
  }
}
