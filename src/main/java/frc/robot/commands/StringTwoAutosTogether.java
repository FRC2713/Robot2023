package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.util.TrajectoryController;

public class StringTwoAutosTogether extends SequentialCommandGroup {

  public StringTwoAutosTogether() {
    addCommands(
        new InstantCommand(
            () -> {
              TrajectoryController.getInstance().changePath(Robot.traj1);
            }),
        new WaitUntilCommand(() -> TrajectoryController.getInstance().isFinished()),
        new InstantCommand(() -> TrajectoryController.getInstance().changePath(Robot.traj2)));
  }
}
