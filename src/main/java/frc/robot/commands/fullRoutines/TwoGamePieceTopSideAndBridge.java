package frc.robot.commands.fullRoutines;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.CommandHelper;
import frc.robot.commands.RaiseElevatorExtendFourBarStopIntake;
import frc.robot.commands.SpinIntakeDrop4BarElevator;
import frc.robot.util.AutoPath;

public class TwoGamePieceTopSideAndBridge extends SequentialCommandGroup {
  // private int i = 0;
  public TwoGamePieceTopSideAndBridge() {
    addCommands(
        new InstantCommand(
            () -> {
              Robot.swerveDrive.resetOdometry(
                  AutoPath.Autos.PART_1.getTrajectory().getInitialHolonomicPose());
            }),
        CommandHelper.stringTrajectoriesTogether(AutoPath.Autos.PART_1.getTrajectory()),
        new SpinIntakeDrop4BarElevator(
            Constants.Elevator.ELEVATOR_MIN_HEIGHT_METERS,
            Constants.FourBarConstants.MAX_ANGLE_RADIANS,
            100),
        new WaitCommand(5),
        new InstantCommand(
            () -> {
              Robot.swerveDrive.resetOdometry(
                  AutoPath.Autos.PART_2.getTrajectory().getInitialHolonomicPose());
            }),
        CommandHelper.stringTrajectoriesTogether(AutoPath.Autos.PART_2.getTrajectory()),
        new RaiseElevatorExtendFourBarStopIntake(
            Constants.Elevator.ELEVATOR_MAX_HEIGHT_METERS,
            Constants.FourBarConstants.MIN_ANGLE_RADIANS),
        new WaitCommand(5),
        new SpinIntakeDrop4BarElevator(
            Constants.Elevator.ELEVATOR_MIN_HEIGHT_METERS,
            Constants.FourBarConstants.MAX_ANGLE_RADIANS,
            100),
        new WaitCommand(5),
        new RaiseElevatorExtendFourBarStopIntake(
            Constants.Elevator.ELEVATOR_MAX_HEIGHT_METERS,
            Constants.FourBarConstants.MIN_ANGLE_RADIANS));
  }
}
