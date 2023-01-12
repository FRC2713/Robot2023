package frc.robot.commands;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.util.TrajectoryController;

public class StringMultipleAutosTogether {
  public static SequentialCommandGroup stringTrajectoriesTogether(
      PathPlannerTrajectory t1, PathPlannerTrajectory t2) {
    return new SequentialCommandGroup(
        new InstantCommand(() -> TrajectoryController.getInstance().changePath(t1)),
        new WaitUntilCommand(() -> TrajectoryController.getInstance().isFinished()),
        new InstantCommand(() -> TrajectoryController.getInstance().changePath(t2)));
  }

  public static SequentialCommandGroup stringTrajectoriesTogether(
      SequentialCommandGroup t1, PathPlannerTrajectory t2) {
    return new SequentialCommandGroup(
        t1,
        new WaitUntilCommand(() -> TrajectoryController.getInstance().isFinished()),
        new InstantCommand(() -> TrajectoryController.getInstance().changePath(t2)));
  }

  public static SequentialCommandGroup followTrajectoryAndWaitForCompletion(
      PathPlannerTrajectory t) {
    return new SequentialCommandGroup(
        new InstantCommand(() -> TrajectoryController.getInstance().changePath(t)),
        new WaitUntilCommand(() -> TrajectoryController.getInstance().isFinished()));
  }

  public static SequentialCommandGroup stringTrajectoriesTogether(
      PathPlannerTrajectory... trajectories) {
    SequentialCommandGroup masterTrajectory =
        new SequentialCommandGroup(
            new InstantCommand(
                () -> TrajectoryController.getInstance().changePath(trajectories[0])));

    for (PathPlannerTrajectory t : trajectories) {
      masterTrajectory.addCommands(followTrajectoryAndWaitForCompletion(t));
    }
    return masterTrajectory;
  }
}
