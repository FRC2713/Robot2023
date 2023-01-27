package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.util.AutoPath.Autos;

public class SpookyScarySkeletons extends SequentialCommandGroup {
  public SpookyScarySkeletons() {
    addCommands(
        new ParallelCommandGroup(
            new InstantCommand(
                () -> {
                  Robot.ele.setTargetHeight(27);
                }),
            new InstantCommand(
                () -> {
                  Robot.four.setAngleDeg(-0.0000002);
                })),
        new WaitUntilCommand(() -> Robot.four.isAtTarget()),
        new WaitUntilCommand(() -> Robot.ele.atTargetHeight()),
        new WaitCommand(2),
        new InstantCommand(
                () -> {
                  Robot.ele.setTargetHeight(0);
                })
            .repeatedly()
            .until(() -> Robot.ele.atTargetHeight()),
        new InstantCommand(
            () ->
                Robot.swerveDrive.resetOdometry(
                    Autos.PART_1.getTrajectory().getInitialHolonomicPose())),
        CommandHelper.stringTrajectoriesTogether(Autos.PART_1.getTrajectory()));
  }
}
// Lovely!
