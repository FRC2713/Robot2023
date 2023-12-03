package frc.robot.util;

import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;
import lib.mechadv.FullStateSwerveTrajectory;

public class ChoreoAutoPaths {
  public static final FullStateSwerveTrajectory TWO_TO_A =
      FullStateSwerveTrajectory.fromFile(
          new File(Filesystem.getDeployDirectory(), "choreo/generated/TwoToA.json"));

  public static void load() {}
}
