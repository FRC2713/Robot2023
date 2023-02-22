package frc.robot.commands.fullRoutines;

import edu.wpi.first.wpilibj2.command.InstantCommand;
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

public class TwoConeOver extends SequentialCommandGroup {

  private SequentialCommandGroup score() {
    return new SequentialCommandGroup(
        FourBar.Commands.extend(),
        Intake.Commands.setRollerVelocityRPM(
            Constants.SuperstructureConstants.SCORE.getRollerRPM(), Robot.gamePieceMode),
        Intake.Commands.setWheelVelocityRPM(
            Constants.SuperstructureConstants.SCORE.getWheelRPM(), Robot.gamePieceMode),
        new WaitCommand(1));
  }

  private SequentialCommandGroup startIntake() {
    return new SequentialCommandGroup(
        FourBar.Commands.extend(),
        Intake.Commands.setRollerVelocityRPM(
            Constants.SuperstructureConstants.INTAKE_UPRIGHT_CONE.getRollerRPM()),
        Intake.Commands.setWheelVelocityRPM(
            Constants.SuperstructureConstants.INTAKE_UPRIGHT_CONE.getWheelRPM()));
  }

  private SequentialCommandGroup stopIntake() {
    SequentialCommandGroup intakeCommand = new SequentialCommandGroup(FourBar.Commands.retract());
    if (Robot.gamePieceMode == GamePieceMode.CONE) {
      intakeCommand.addCommands(
          Intake.Commands.setRollerVelocityRPM(500), Intake.Commands.setWheelVelocityRPM(500));
    } else {
      intakeCommand.addCommands(
          Intake.Commands.setRollerVelocityRPM(0), Intake.Commands.setWheelVelocityRPM(0));
    }
    ;
    return intakeCommand;
  }

  public TwoConeOver() {
    addCommands(
        new InstantCommand(
            () -> {
              Robot.swerveDrive.resetOdometry(
                  Autos.ONE_TO_A.getTrajectory().getInitialHolonomicPose());
              Robot.gamePieceMode = GamePieceMode.CONE;
            }),
        FourBar.Commands.retract(),
        Elevator.Commands.elevatorConeMidScoreAndWait(),
        score(),
        stopIntake(),
        Elevator.Commands.elevatorConeFloorUpIntakeAndWait(),
        startIntake(),
        SwerveSubsystem.Commands.stringTrajectoriesTogether(Autos.ONE_TO_A.getTrajectory()),
        new WaitUntilCommand(() -> TrajectoryController.getInstance().isFinished()),
        stopIntake(),
        SwerveSubsystem.Commands.stringTrajectoriesTogether(Autos.A_TO_THREE.getTrajectory()),
        new WaitUntilCommand(() -> TrajectoryController.getInstance().isFinished()),
        Elevator.Commands.elevatorConeMidScoreAndWait(),
        score(),
        stopIntake(),
        Elevator.Commands.setTargetHeightAndWait(Constants.zero));
  }
}
