package frc.robot.util;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;

public class AutoPath {
  public static final double fieldLength = Units.inchesToMeters(651.25);
  public static final double fieldWidth = Units.inchesToMeters(315.5);

  public enum Autos {
    // Cone Paths (Paths to/from grids with cone goals)
    A_TO_FIVE("cargoAtogrid5"),
    A_TO_FOUR("cargoAtogrid4"),
    A_TO_THREE("cargoAtogrid3"),
    D_TO_SEVEN("cargoDtogrid7"),
    ONE_TO_A("grid1tocargoA"),
    THREE_TO_B("grid3tocargoB"),
    FOUR_TO_B("grid4tocargoB"),
    NINE_TO_D("grid9tocargoD"),

    // Cube Paths (Paths to/from grids with cube goals)
    A_TO_TWO("cargoAtogrid2"),
    B_TO_TWO("cargoBtogrid2"),
    TWO_TO_A("grid2tocargoA"),
    TWO_TO_B("grid2tocargoB"),
    FIVE_TO_B("grid5tocargoB"),

    // Misc Paths (look man I can only make so many categories)
    A_TO_BRIDGE("cargoAtobridge");
    private PathPlannerTrajectory blueTrajectory, redTrajectory;

    private Autos(String filename, double maxVel, double maxAccel) {
      try {
        blueTrajectory = PathPlanner.loadPath(filename, new PathConstraints(maxVel, maxAccel));
        redTrajectory = ReflectedTransform.reflectiveTransformTrajectory(blueTrajectory);
      } catch (NullPointerException notAgain) {
        System.out.println(filename + "is not found.");
        RedHawkUtil.ErrHandler.getInstance()
            .addError(filename + "Is Not Found. This is really bad guys this file is not found");
      }
    }

    private Autos(String filename) {
      this(
          filename,
          Constants.DriveConstants.MAX_SWERVE_VEL,
          Constants.DriveConstants.MAX_SWERVE_ACCEL);
    }

    public PathPlannerTrajectory getTrajectory() {
      return DriverStation.getAlliance() == Alliance.Blue
          ? this.blueTrajectory
          : this.redTrajectory;
    }
  }
}
