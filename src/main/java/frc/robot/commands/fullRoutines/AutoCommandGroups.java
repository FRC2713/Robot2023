package frc.robot.commands.fullRoutines;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.Robot;
import frc.robot.Robot.GamePieceMode;
import frc.robot.subsystems.elevatorIO.Elevator;
import frc.robot.subsystems.fourBarIO.FourBar;
import frc.robot.subsystems.intakeIO.Intake;
import frc.robot.util.SuperstructureConfig;

public class AutoCommandGroups {

  public static Command score(
      SuperstructureConfig config, double waitAtEnd, boolean waitForFourbar) {
    return Commands.sequence(
        new InstantCommand(() -> Robot.intake.setScoring(true)),
        prepScore(config, waitForFourbar),
        Intake.Commands.score(),
        new WaitCommand(waitAtEnd));
  }

  // Just scores at whatever height we're at and immediatly begin intaking
  public static Command scoreLowNoPrep() {
    return Commands.sequence(Intake.Commands.score(), new WaitCommand(0.5), startIntake(false));
  }

  public static Command prepScore(SuperstructureConfig config, boolean waitForFourbar) {
    return Commands.sequence(
        Elevator.Commands.setToHeightAndWait(config),
        waitForFourbar
            ? FourBar.Commands.setAngleDegAndWait(config)
            : FourBar.Commands.setToAngle(config));
  }

  // Spit out a cube, not neccesarily to score
  public static Command spitCube() {
    return new ParallelCommandGroup(
        new InstantCommand(() -> Robot.intake.setScoring(true)),
        FourBar.Commands.setToAngle(SuperstructureConstants.INTAKE_CUBE.getFourBarPosition()),
        Intake.Commands.setBottomVelocityRPM(
            SuperstructureConstants.SCORE_CUBE_HIGH.getBottomRPM() * 1.5),
        Intake.Commands.setTopVelocityRPM(
            SuperstructureConstants.SCORE_CUBE_HIGH.getTopRPM() * 1.5));
  }

  public static Command startIntake(boolean waitForFourbar) {
    SuperstructureConfig intakeConfig =
        Robot.gamePieceMode == GamePieceMode.CUBE
            ? SuperstructureConstants.INTAKE_CUBE
            : SuperstructureConstants.INTAKE_TIPPED_CONE;

    return new ParallelCommandGroup(
        new InstantCommand(() -> Robot.intake.setScoring(false)),
        waitForFourbar
            ? FourBar.Commands.setAngleDegAndWait(intakeConfig.getFourBarPosition())
            : FourBar.Commands.setToAngle(intakeConfig.getFourBarPosition()),
        Intake.Commands.setBottomVelocityRPM(intakeConfig.getBottomRPM()),
        Intake.Commands.setTopVelocityRPM(intakeConfig.getTopRPM()));
  }

  public static Command stopIntake() {
    SuperstructureConfig holdConfig =
        Robot.gamePieceMode == GamePieceMode.CUBE
            ? SuperstructureConstants.HOLD_CUBE
            : SuperstructureConstants.HOLD_CONE;

    return new ParallelCommandGroup(
        new InstantCommand(() -> Robot.intake.setScoring(false)),
        Intake.Commands.setBottomVelocityRPM(holdConfig.getBottomRPM()),
        Intake.Commands.setTopVelocityRPM(holdConfig.getTopRPM()),
        FourBar.Commands.retract());
  }
}
