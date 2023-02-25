package frc.robot.commands.fullRoutines;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Robot.GamePieceMode;
import frc.robot.subsystems.elevatorIO.Elevator;
import frc.robot.subsystems.fourBarIO.FourBar;
import frc.robot.subsystems.intakeIO.Intake;
import frc.robot.subsystems.swerveIO.SwerveSubsystem;
import frc.robot.util.AutoPath.Autos;
import frc.robot.util.TrajectoryController;

public class TwoCubeOver extends SequentialCommandGroup {

  private Command score() {
    return new SequentialCommandGroup(
        FourBar.Commands.score(), Intake.Commands.score(), new WaitCommand(1));
  }

  private Command startIntake() {
    return new ConditionalCommand(
        new ParallelCommandGroup(
            FourBar.Commands.setToAngle(
                Constants.SuperstructureConstants.INTAKE_CUBE.getFourBarPosition()),
            Intake.Commands.setRollerVelocityRPM(
                Constants.SuperstructureConstants.INTAKE_CUBE.getRollerRPM()),
            Intake.Commands.setWheelVelocityRPM(
                Constants.SuperstructureConstants.INTAKE_CUBE.getWheelRPM())),
        new ParallelCommandGroup(
            FourBar.Commands.setToAngle(
                Constants.SuperstructureConstants.INTAKE_UPRIGHT_CONE.getFourBarPosition()),
            Intake.Commands.setRollerVelocityRPM(
                Constants.SuperstructureConstants.INTAKE_UPRIGHT_CONE.getRollerRPM()),
            Intake.Commands.setWheelVelocityRPM(
                Constants.SuperstructureConstants.INTAKE_UPRIGHT_CONE.getWheelRPM())),
        () -> Robot.gamePieceMode == GamePieceMode.CUBE);
  }

  private Command stopIntake() {
    return new ConditionalCommand(
        new ParallelCommandGroup(
            Intake.Commands.setRollerVelocityRPM(0),
            Intake.Commands.setWheelVelocityRPM(0),
            FourBar.Commands.retract()),
        new ParallelCommandGroup(
            Intake.Commands.setRollerVelocityRPM(-500),
            Intake.Commands.setWheelVelocityRPM(-500),
            FourBar.Commands.retract()),
        () -> Robot.gamePieceMode == GamePieceMode.CUBE);
  }

  public TwoCubeOver() {
    addCommands(
        new InstantCommand(
            () -> {
              Robot.swerveDrive.resetOdometry(
                  Autos.TWO_TO_A.getTrajectory().getInitialHolonomicPose());
              Robot.gamePieceMode = GamePieceMode.CUBE;
            }),
        FourBar.Commands.retract(),
        Elevator.Commands.elevatorCubeLowScoreAndWait(),
        score(),
        stopIntake(),
        Elevator.Commands.elevatorCubeFloorIntakeAndWait(),
        startIntake(),
        SwerveSubsystem.Commands.stringTrajectoriesTogether(Autos.TWO_TO_A.getTrajectory()),
        new WaitUntilCommand(() -> TrajectoryController.getInstance().isFinished()),
        stopIntake(),
        SwerveSubsystem.Commands.stringTrajectoriesTogether(Autos.A_TO_TWO.getTrajectory()),
        new WaitUntilCommand(() -> TrajectoryController.getInstance().isFinished()),
        Elevator.Commands.elevatorCubeMidScoreAndWait(),
        score(),
        stopIntake(),
        Elevator.Commands.setToHeight(Constants.zero));
  }
}