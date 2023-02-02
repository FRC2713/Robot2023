package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.subsystems.swerveIO.SwerveSubsystem;
import frc.robot.util.AutoPath.Autos;

public class SpookyScarySkeletons extends SequentialCommandGroup {
  public SpookyScarySkeletons() {
    addCommands(
        new ParallelCommandGroup(
            new InstantCommand(
                () -> {
                  Robot.elevator.setTargetHeight(27);
                }),
            new InstantCommand(
                () -> {
                  Robot.fourBar.setAngleDeg(-0.0000002);
                })),
        new WaitUntilCommand(() -> Robot.fourBar.isAtTarget()),
        new WaitUntilCommand(() -> Robot.elevator.atTargetHeight()),
        new WaitCommand(2),
        new InstantCommand(
                () -> {
                  Robot.elevator.setTargetHeight(0);
                })
            .repeatedly()
            .until(() -> Robot.elevator.atTargetHeight()),
        new InstantCommand(
            () ->
                Robot.swerveDrive.resetOdometry(
                    Autos.A_TO_THREE.getTrajectory().getInitialHolonomicPose())),
        SwerveSubsystem.Commands.stringTrajectoriesTogether(Autos.A_TO_THREE.getTrajectory()));
  }
}
// Lovely!
