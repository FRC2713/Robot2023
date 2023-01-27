package frc.robot.util;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class AutoPath {
  public static final double fieldLength = Units.inchesToMeters(651.25);
  public static final double fieldWidth = Units.inchesToMeters(315.5);

  public enum Autos {
    GO_TO_FIRST_CARGO("dock"),
    GO_TO_GRID("gotogrid"),
    GO_TO_SECOND_CARGO("goto2ndcargo"),
    GO_TO_GRID_TWO("gotogrid2"),
    DOCK("dock"),
    ONE_TO_A("dock"),
    A_TO_THREE("dock");
    private PathPlannerTrajectory blueTrajectory, redTrajectory;

    private Autos(String filename) {
      try {
        this.blueTrajectory =
            PathPlanner.loadPath(filename, PathPlanner.getConstraintsFromPath(filename));
        this.redTrajectory = ReflectedTransform.reflectiveTransformTrajectory(blueTrajectory);
      } catch (NullPointerException ex) {
        this.blueTrajectory = PathPlanner.loadPath(filename, new PathConstraints(4, 3));
        this.redTrajectory = ReflectedTransform.reflectiveTransformTrajectory(blueTrajectory);
      }
    }

    public PathPlannerTrajectory getTrajectory() {
      return DriverStation.getAlliance() == Alliance.Blue
          ? this.blueTrajectory
          : this.redTrajectory;
    }
  }
}
