package frc.robot.commands.fullRoutines;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
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

public class CommandGroups {
  public static Command score(SuperstructureConfig config) {
    return Commands.sequence(
        new InstantCommand(() -> Robot.intake.setScoring(true)),
        prepScore(config),
        Intake.Commands.score(),
        new WaitCommand(0.75));
  }

  public static Command prepScore(SuperstructureConfig config) {
    return Commands.sequence(
        Elevator.Commands.setToHeightAndWait(config), FourBar.Commands.setAngleDegAndWait(config));
  }

  public static Command startIntake() {
    return new ConditionalCommand(
        new ParallelCommandGroup(
            new InstantCommand(() -> Robot.intake.setScoring(false)),
            FourBar.Commands.setToAngle(SuperstructureConstants.INTAKE_CUBE.getFourBarPosition()),
            Intake.Commands.setBottomVelocityRPM(
                SuperstructureConstants.INTAKE_CUBE.getBottomRPM()),
            Intake.Commands.setTopVelocityRPM(SuperstructureConstants.INTAKE_CUBE.getTopRPM())),
        new ParallelCommandGroup(
            new InstantCommand(() -> Robot.intake.setScoring(false)),
            FourBar.Commands.setToAngle(
                SuperstructureConstants.INTAKE_TIPPED_CONE.getFourBarPosition()),
            Intake.Commands.setBottomVelocityRPM(
                SuperstructureConstants.INTAKE_TIPPED_CONE.getBottomRPM()),
            Intake.Commands.setTopVelocityRPM(
                SuperstructureConstants.INTAKE_TIPPED_CONE.getTopRPM())),
        () -> Robot.gamePieceMode == GamePieceMode.CUBE);
  }

  public static Command stopIntake() {
    return new ConditionalCommand(
        new ParallelCommandGroup(
            new InstantCommand(() -> Robot.intake.setScoring(false)),
            Intake.Commands.setBottomVelocityRPM(SuperstructureConstants.HOLD_CUBE.getBottomRPM()),
            Intake.Commands.setTopVelocityRPM(SuperstructureConstants.HOLD_CUBE.getTopRPM()),
            FourBar.Commands.retract()),
        new ParallelCommandGroup(
            new InstantCommand(() -> Robot.intake.setScoring(false)),
            Intake.Commands.setBottomVelocityRPM(SuperstructureConstants.HOLD_CONE.getBottomRPM()),
            Intake.Commands.setTopVelocityRPM(SuperstructureConstants.HOLD_CONE.getTopRPM()),
            FourBar.Commands.retract()),
        () -> Robot.gamePieceMode == GamePieceMode.CUBE);
  }
}
