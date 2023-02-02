package frc.robot.util;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class AutoPath {
  public static final double fieldLength = Units.inchesToMeters(651.25);
  public static final double fieldWidth = Units.inchesToMeters(315.5);

  public enum Autos {
    DOCK("dock"),
    A_TO_FIVE("cargoAtogrid5", 4, 3),
    A_TO_THREE("cargoAtogrid3", 4, 3),
    D_TO_SEVEN("cargoDtogrid7", 4, 3),
    ONE_TO_A("grid1tocargoA", 4, 3),
    THREE_TO_B("grid3tocargoB", 4, 3),
    NINE_TO_D("grid9tocargoD", 4, 3);

    private PathPlannerTrajectory trajectory;

    private Autos(String filename, double maxVel, double maxAccel) {
      try {
        this.trajectory =
            ReflectedTransform.reflectiveTransformTrajectory(
                PathPlanner.loadPath(filename, new PathConstraints(maxVel, maxAccel)));
      } catch (NullPointerException notAgain) {
        System.out.println(filename + "is not found.");
        RedHawkUtil.ErrHandler.getInstance()
            .addError(filename + "Is Not Found. This is really bad guys this file is not found");
      }
    }

    private Autos(String filename) {
      this(
          filename, Constants.DriveConstants.maxSwerveVel, Constants.DriveConstants.maxSwerveAccel);
    }

    public PathPlannerTrajectory getTrajectory() {
      return DriverStation.getAlliance() == Alliance.Blue
          ? this.blueTrajectory
          : this.redTrajectory;
    }
  }
}
