package frc.robot.commands.fullRoutines;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.GetOnBridge;
import frc.robot.subsystems.elevatorIO.Elevator;
import frc.robot.subsystems.fourBarIO.FourBar;
import frc.robot.subsystems.swerveIO.SwerveSubsystem;
import frc.robot.util.AutoPath.Autos;

public class OneToAToThreeToBridge extends SequentialCommandGroup {
  public OneToAToThreeToBridge() {
    addCommands(
        Elevator.Commands.setToHeight(30),
        FourBar.Commands.setToAngle(Constants.FourBarConstants.MAX_ANGLE_RADIANS),
        new InstantCommand(
            () -> {
              Robot.swerveDrive.resetOdometry(
                  Autos.ONE_TO_A.getTrajectory().getInitialHolonomicPose());
            }),
        new WaitUntilCommand(() -> Robot.ele.atTargetHeight()),
        new ParallelCommandGroup(
            SwerveSubsystem.Commands.stringTrajectoriesTogether(Autos.ONE_TO_A.getTrajectory()),
            Elevator.Commands.setToHeight(Constants.zero),
            FourBar.Commands.setToAngle(Constants.FourBarConstants.MIN_ANGLE_RADIANS)),
        new ParallelCommandGroup(
            SwerveSubsystem.Commands.stringTrajectoriesTogether(Autos.A_TO_FIVE.getTrajectory()),
            Elevator.Commands.setToHeight(30),
            FourBar.Commands.setToAngle(Constants.FourBarConstants.MAX_ANGLE_RADIANS)),
        Elevator.Commands.setToHeight(Constants.zero),
        FourBar.Commands.setToAngle(Constants.FourBarConstants.MIN_ANGLE_RADIANS),
        new WaitUntilCommand(() -> Robot.ele.atTargetHeight()),
        new GetOnBridge());
  }
}
