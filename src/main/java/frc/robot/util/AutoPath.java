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
    A_TO_THREE("cargoAtogrid3"),
    A_TO_FOUR("cargoAtogrid4"),
    B_TO_THREE("cargoBtogrid3"),
    B_TO_FOUR("cargoBtogrid4"),
    D_TO_SEVEN("cargoDtogrid7"),
    ONE_TO_A("grid1tocargoA"),
    REVERSE_ONE_TO_A("reversegrid1tocargoA"),
    TRAJ_MOBILITY("slap6tomobility", Units.feetToMeters(16), Units.feetToMeters(3)),
    NO_SLAP_TRAJ_MOBILITY("grid6tomobility", Units.feetToMeters(16), Units.feetToMeters(3)),
    MOBILITY_TO_CHARGE("mobilitytocharge", Units.feetToMeters(16), Units.feetToMeters(3)),
    THREE_TO_A("grid3tocargoA"),
    THREE_TO_B("grid3tocargoB"),
    FOUR_TO_B("grid4tocargoB"),
    SEVEN_TO_D("grid7tocargoD"),
    NINE_TO_D("grid9tocargoD"),

    // Cube Paths (Paths to/from grids with cube goals)
    A_TO_TWO("cargoAtogrid2"),
    A_TO_FIVE("cargoAtogrid5"),
    B_TO_TWO("cargoBtogrid2"),
    B_TO_FIVE("cargoBtogrid5"),
    D_TO_EIGHT("cargoDtogrid8"),
    TWO_TO_A("grid2tocargoA"),
    TWO_TO_B("grid2tocargoB"),
    FIVE_TO_B("grid5tocargoB"),
    EIGHT_TO_D("grid8tocargoD"),

    // community autos
    D_TO_COMMUNITY("cargoDtocommunity"),
    COMMUNITY_TO_C("communitytocargoC"),
    C_TO_COMMUNITY("cargoCtocommunity"),

    // Misc Paths (look man I can only make so many categories)
    A_TO_BRIDGE("cargoAtobridge"),
    B_TO_BRIDGE("cargoBtobridge"),
    FIVE_ON_BRIDGE("grid5tobridge", Units.feetToMeters(16), Units.feetToMeters(1));
    private PathPlannerTrajectory blueTrajectory, redTrajectory;

    private Autos(String filename, double maxVel, double maxAccel) {
      try {
        blueTrajectory = PathPlanner.loadPath(filename, new PathConstraints(maxVel, maxAccel));
        redTrajectory = ReflectedTransform.reflectiveTransformTrajectory(blueTrajectory);

        System.out.println(
            "Red "
                + this.name()
                + " init holo pose -> "
                + redTrajectory.getInitialHolonomicPose().getRotation().getDegrees());
      } catch (NullPointerException notAgain) {
        System.out.println(filename + "is not found.");
        RedHawkUtil.ErrHandler.getInstance()
            .addError(filename + "Is Not Found. This is really bad guys this file is not found");
      }
    }

    private Autos(String filename) {
      this(
          filename,
          Constants.DriveConstants.MAX_SWERVE_VEL_AUTO,
          Constants.DriveConstants.MAX_SWERVE_ACCEL);
    }

    private void clear() {
      this.redTrajectory = null;
      this.blueTrajectory = null;
    }

    public static void clearAll() {
      for (Autos auto : Autos.values()) {
        auto.clear();
      }
    }

    public PathPlannerTrajectory getTrajectory() {
      return DriverStation.getAlliance() == Alliance.Blue
          ? this.blueTrajectory
          : this.redTrajectory;
    }
  }
}
