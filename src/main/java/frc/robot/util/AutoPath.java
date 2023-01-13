package frc.robot.util;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

public class AutoPath {
  public enum Autos {
    PART_1("load1stcargo"),
    PART_2("getonthebridge");

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
