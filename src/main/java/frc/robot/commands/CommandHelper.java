package frc.robot.commands;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.util.TrajectoryController;

public class CommandHelper {
  public static SequentialCommandGroup followTAndWait(PathPlannerTrajectory T) {
    return new SequentialCommandGroup(
        new InstantCommand(() -> TrajectoryController.getInstance().changePath(T)),
        new WaitUntilCommand(() -> TrajectoryController.getInstance().isFinished()));
  }

  public static SequentialCommandGroup stringTrajectoriesTogether(
      PathPlannerTrajectory... trajectories) {
    SequentialCommandGroup masterTrajectory =
        new SequentialCommandGroup(
            new InstantCommand(
                () -> TrajectoryController.getInstance().changePath(trajectories[0])));

    for (PathPlannerTrajectory t : trajectories) {
      masterTrajectory.addCommands(followTAndWait(t));
    }
    return masterTrajectory;
  }
}
