package frc.robot.commands.fullRoutines;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.util.ChoreoAutoPaths;
import lib.mechadv.CustomTrajectoryController;

public class ChoreoTestAuto extends SequentialCommandGroup {
  public ChoreoTestAuto() {
    addCommands(
        new InstantCommand(
            () -> {
              CustomTrajectoryController.getInstance().changePath(ChoreoAutoPaths.TWO_TO_A);
              Robot.swerveDrive.resetOdometry(
                  ChoreoAutoPaths.TWO_TO_A.getStates().stream()
                      .findFirst()
                      .orElseThrow()
                      .getPose());
            }),
        new WaitUntilCommand(() -> CustomTrajectoryController.getInstance().isFinished()));
  }
}
