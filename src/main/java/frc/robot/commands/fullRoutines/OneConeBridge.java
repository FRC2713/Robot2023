package frc.robot.commands.fullRoutines;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.Robot;
import frc.robot.Robot.GamePieceMode;
import frc.robot.commands.PIDOnBridge;
import frc.robot.subsystems.elevatorIO.Elevator;
import frc.robot.subsystems.intakeIO.Intake;
import frc.robot.util.AutoPath.Autos;

public class OneConeBridge extends SequentialCommandGroup {

  private double delayAfterScoring = 0.5;
  private boolean waitForFourbarDuringScoring = true;

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
        AutoCommandGroups.score(
            SuperstructureConstants.SCORE_CONE_HIGH,
            delayAfterScoring,
            waitForFourbarDuringScoring),

        // Move to Charge Station
        AutoCommandGroups.stopIntake().repeatedly().until(() -> Robot.fourBar.isAtTarget()),
        Elevator.Commands.setToHeight(0),
        new PIDOnBridge(true));
  }
}
