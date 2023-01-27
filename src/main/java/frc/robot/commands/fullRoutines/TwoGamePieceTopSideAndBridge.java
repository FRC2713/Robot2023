package frc.robot.commands.fullRoutines;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.RaiseElevatorExtendFourBarStopIntake;
import frc.robot.commands.SpinIntakeDrop4BarElevator;
import frc.robot.subsystems.swerveIO.SwerveSubsystem;
import frc.robot.util.AutoPath;

public class TwoGamePieceTopSideAndBridge extends SequentialCommandGroup {
  public TwoGamePieceTopSideAndBridge() {
    addCommands(
        // // Go to first cargo
        // new InstantCommand(
        //     () -> {
        //       Robot.swerveDrive.resetOdometry(
        //           AutoPath.Autos.GO_TO_FIRST_CARGO.getTrajectory().getInitialHolonomicPose());
        //     }),
        // CommandHelper.stringTrajectoriesTogether(AutoPath.Autos.GO_TO_FIRST_CARGO.getTrajectory()),
        // // Get cargo
        // new SpinIntakeDrop4BarElevator(
        //     Constants.Elevator.ELEVATOR_MIN_HEIGHT_METERS,
        //     Constants.FourBarConstants.MAX_ANGLE_RADIANS,
        //     500),

        // Go to grid
        new InstantCommand(
            () -> {
              Robot.swerveDrive.resetOdometry(
                  AutoPath.Autos.GO_TO_GRID.getTrajectory().getInitialHolonomicPose());
            }),
        SwerveSubsystem.Commands.stringTrajectoriesTogether(
            AutoPath.Autos.GO_TO_GRID.getTrajectory()),
        // Score game peice
        new RaiseElevatorExtendFourBarStopIntake(
            Constants.ElevatorConstants.ELEVATOR_MAX_HEIGHT_METERS,
            Constants.FourBarConstants.MIN_ANGLE_RADIANS),
        new WaitCommand(0.5),
        // Go to second cargo
        new InstantCommand(
            () -> {
              Robot.swerveDrive.resetOdometry(
                  AutoPath.Autos.GO_TO_FIRST_CARGO.getTrajectory().getInitialHolonomicPose());
            }),
        SwerveSubsystem.Commands.stringTrajectoriesTogether(
            AutoPath.Autos.GO_TO_FIRST_CARGO.getTrajectory()),
        // Pick up second cargo
        new SpinIntakeDrop4BarElevator(
            Constants.ElevatorConstants.ELEVATOR_MIN_HEIGHT_METERS,
            Constants.FourBarConstants.MAX_ANGLE_RADIANS,
            500),
        // Go to grid second time
        new InstantCommand(
            () -> {
              Robot.swerveDrive.resetOdometry(
                  AutoPath.Autos.GO_TO_GRID_TWO.getTrajectory().getInitialHolonomicPose());
            }),
        SwerveSubsystem.Commands.stringTrajectoriesTogether(
            AutoPath.Autos.GO_TO_GRID_TWO.getTrajectory()),
        // Score second cargo
        new RaiseElevatorExtendFourBarStopIntake(
            Constants.ElevatorConstants.ELEVATOR_MAX_HEIGHT_METERS,
            Constants.FourBarConstants.MIN_ANGLE_RADIANS),
        new WaitCommand(0.5),
        // Dock
        new InstantCommand(
            () -> {
              Robot.swerveDrive.resetOdometry(
                  AutoPath.Autos.DOCK.getTrajectory().getInitialHolonomicPose());
            }),
        SwerveSubsystem.Commands.stringTrajectoriesTogether(AutoPath.Autos.DOCK.getTrajectory()));
  }
}
