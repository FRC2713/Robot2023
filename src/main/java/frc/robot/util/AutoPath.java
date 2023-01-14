package frc.robot.util;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

public class AutoPath {
  public enum Autos {
    PART_1("goto1stcargo"),
    PART_2("backtogrid"),
    PART_3("getonthebridge");

    private String filename;

    private Autos(String filename) {
      this.filename = filename;
    }

    public PathPlannerTrajectory getTrajectory() {
      PathPlannerTrajectory trajectory =
          PathPlanner.loadPath(filename, PathPlanner.getConstraintsFromPath(filename));
      return trajectory;
    }
  }
}
