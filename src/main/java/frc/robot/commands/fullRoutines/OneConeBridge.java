package frc.robot.commands.fullRoutines;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.Robot;
import frc.robot.Robot.GamePieceMode;
import frc.robot.commands.PIDOnBridgeExperimental;
import frc.robot.subsystems.elevatorIO.Elevator;
import frc.robot.subsystems.fourBarIO.FourBar;
import frc.robot.subsystems.intakeIO.Intake;
import frc.robot.util.AutoPath.Autos;
import frc.robot.util.SuperstructureConfig;

public class OneConeBridge extends SequentialCommandGroup {

  private double delayAfterScoring = 0.5;
  private boolean waitForFourbarDuringScoring = true;

  public Command prepScore(SuperstructureConfig config, boolean waitForFourbar) {
    return Commands.sequence(
        Elevator.Commands.setToHeightAndWait(config),
        waitForFourbar
            ? FourBar.Commands.setAngleDegAndWait(config)
            : FourBar.Commands.setToAngle(config));
  }

  public Command score(SuperstructureConfig config, double waitAtEnd, boolean waitForFourbar) {
    return Commands.sequence(
        new InstantCommand(() -> Robot.intake.setScoring(true)),
        prepScore(config, waitForFourbar),
        Intake.Commands.score(),
        new WaitCommand(waitAtEnd));
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

  public OneConeBridge() {
    addCommands(
        // Score preload
        new InstantCommand(
            () -> {
              Robot.swerveDrive.resetOdometry(
                  Autos.FIVE_TO_B.getTrajectory().getInitialHolonomicPose());
              Robot.gamePieceMode = GamePieceMode.CONE;
              Robot.fourBar.reseed();
            }),
        Intake.Commands.setBottomVelocityRPM(SuperstructureConstants.HOLD_CONE.getBottomRPM()),
        Intake.Commands.setTopVelocityRPM(SuperstructureConstants.HOLD_CONE.getTopRPM()),
        score(
            SuperstructureConstants.SCORE_CONE_HIGH,
            delayAfterScoring,
            waitForFourbarDuringScoring),

        // Move to Charge Station
        stopIntake().repeatedly().until(() -> Robot.fourBar.isAtTarget()),
        Elevator.Commands.setToHeight(0),
        new PIDOnBridgeExperimental(true));
  }
}
