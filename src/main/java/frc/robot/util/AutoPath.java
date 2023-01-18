package frc.robot.util;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.util.Units;

public class AutoPath {
  public static final double fieldLength = Units.inchesToMeters(651.25);
  public static final double fieldWidth = Units.inchesToMeters(315.5);

  public enum Autos {
    PART_1("goto1stcargo"),
    PART_2("backtogrid"),
    PART_3("getonthebridge");

    private PathPlannerTrajectory trajectory;

    private Autos(String filename) {
      this.trajectory =
          ReflectedTransform.reflectiveTransformTrajectory(
              PathPlanner.loadPath(filename, PathPlanner.getConstraintsFromPath(filename)));
    }

    public PathPlannerTrajectory getTrajectory() {
      return trajectory;
    }
  }
}
