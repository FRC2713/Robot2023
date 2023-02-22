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
import frc.robot.util.AutoPath;
import frc.robot.util.AutoPath.Autos;
import frc.robot.util.TrajectoryController;

public class TwoConeUnder extends SequentialCommandGroup {
  private SequentialCommandGroup score() {
    return new SequentialCommandGroup(
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

  public TwoConeUnder() {
    addCommands(
        new InstantCommand(
            () -> {
              Robot.swerveDrive.resetOdometry(
                  AutoPath.Autos.NINE_TO_D.getTrajectory().getInitialHolonomicPose());
              Robot.gamePieceMode = GamePieceMode.CONE;
            }),
        FourBar.Commands.retract(),
        Elevator.Commands.elevatorConeMidScoreAndWait(),
        score(),
        stopIntake(),
        Elevator.Commands.elevatorConeFloorUpIntakeAndWait(),
        SwerveSubsystem.Commands.stringTrajectoriesTogether(Autos.NINE_TO_D.getTrajectory()),
        startIntake(),
        new WaitUntilCommand(() -> TrajectoryController.getInstance().isFinished()),
        stopIntake(),
        SwerveSubsystem.Commands.stringTrajectoriesTogether(Autos.D_TO_SEVEN.getTrajectory()),
        new WaitUntilCommand(() -> TrajectoryController.getInstance().isFinished()),
        Elevator.Commands.elevatorConeMidScoreAndWait(),
        score(),
        stopIntake(),
        Elevator.Commands.setTargetHeightAndWait(Constants.zero));
  }
}
