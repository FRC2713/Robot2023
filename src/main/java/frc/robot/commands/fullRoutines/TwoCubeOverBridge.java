package frc.robot.commands.fullRoutines;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.Robot;
import frc.robot.Robot.GamePieceMode;
import frc.robot.commands.PIDOnBridge;
import frc.robot.subsystems.elevatorIO.Elevator;
import frc.robot.subsystems.intakeIO.Intake;
import frc.robot.subsystems.swerveIO.SwerveSubsystem;
import frc.robot.util.AutoPath.Autos;

public class TwoCubeOverBridge extends SequentialCommandGroup {

  private double delayAfterScoring = 0.5;
  private boolean waitForFourbarDuringScoring = false;
  private boolean waitForFourbarDuringIntaking = false;

  public TwoCubeOverBridge() {
    addCommands(
        // Score preload and obtain game piece
        new InstantCommand(
            () -> {
              Robot.swerveDrive.resetOdometry(
                  Autos.TWO_TO_A.getTrajectory().getInitialHolonomicPose());
              Robot.gamePieceMode = GamePieceMode.CUBE;
              Robot.intake.setScoring(true);
              Robot.fourBar.reseed();
            }),
        Commands.parallel(
            Commands.sequence(
                Intake.Commands.score(),
                new WaitCommand(0.5),
                AutoCommandGroups.startIntake(waitForFourbarDuringIntaking)),
            Elevator.Commands.setToHeightAndWait(SuperstructureConstants.INTAKE_CUBE),
            SwerveSubsystem.Commands.stringTrajectoriesTogether(Autos.TWO_TO_A.getTrajectory())),

        // Score game piece
        Commands.parallel(
            AutoCommandGroups.prepScore(
                SuperstructureConstants.SCORE_CUBE_MID, waitForFourbarDuringScoring),
            SwerveSubsystem.Commands.stringTrajectoriesTogether(Autos.A_TO_FIVE.getTrajectory())),
        AutoCommandGroups.score(
            SuperstructureConstants.SCORE_CUBE_MID, delayAfterScoring, waitForFourbarDuringScoring),

        // Balance on charge station
        Commands.parallel(
            AutoCommandGroups.stopIntake(),
            Elevator.Commands.setToHeight(0),
            new PIDOnBridge(true)));
  }
}
