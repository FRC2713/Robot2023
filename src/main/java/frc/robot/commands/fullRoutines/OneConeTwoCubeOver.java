package frc.robot.commands.fullRoutines;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.Robot;
import frc.robot.Robot.GamePieceMode;
import frc.robot.subsystems.elevatorIO.Elevator;
import frc.robot.subsystems.intakeIO.Intake;
import frc.robot.subsystems.swerveIO.SwerveSubsystem;
import frc.robot.util.AutoPath.Autos;

public class OneConeTwoCubeOver extends SequentialCommandGroup {
  private double delayAfterScoring = 0.5;
  private boolean waitForFourbarDuringScoring = true;
  private boolean waitForFourbarDuringIntaking = false;

  public OneConeTwoCubeOver() {
    addCommands(
        // Score preload
        new InstantCommand(
            () -> {
              Robot.swerveDrive.resetOdometry(
                  Autos.ONE_TO_A.getTrajectory().getInitialHolonomicPose());
              Robot.gamePieceMode = GamePieceMode.CONE;
              Robot.fourBar.reseed();
            }),
        Intake.Commands.setBottomVelocityRPM(SuperstructureConstants.HOLD_CONE.getBottomRPM()),
        Intake.Commands.setTopVelocityRPM(SuperstructureConstants.HOLD_CONE.getTopRPM()),
        AutoCommandGroups.score(
            SuperstructureConstants.SCORE_CONE_HIGH,
            delayAfterScoring,
            waitForFourbarDuringScoring),
        AutoCommandGroups.stopIntake(),

        // Get and score cube
        Elevator.Commands.setToHeightAndWait(SuperstructureConstants.INTAKE_CUBE),
        Commands.parallel(
            SwerveSubsystem.Commands.stringTrajectoriesTogether(Autos.ONE_TO_A.getTrajectory()),
            new InstantCommand(() -> Robot.gamePieceMode = GamePieceMode.CUBE),
            Commands.sequence(
                new WaitCommand(0.5), AutoCommandGroups.startIntake(waitForFourbarDuringIntaking))),
        AutoCommandGroups.stopIntake(),
        Commands.sequence(
            SwerveSubsystem.Commands.stringTrajectoriesTogether(Autos.A_TO_TWO.getTrajectory()),
            AutoCommandGroups.prepScore(
                SuperstructureConstants.SCORE_CUBE_HIGH, waitForFourbarDuringScoring)),
        AutoCommandGroups.score(
            SuperstructureConstants.SCORE_CUBE_HIGH,
            delayAfterScoring,
            waitForFourbarDuringScoring),
        AutoCommandGroups.stopIntake(),

        // Get and score cube
        Elevator.Commands.setToHeightAndWait(SuperstructureConstants.INTAKE_CUBE),
        Commands.parallel(
            SwerveSubsystem.Commands.stringTrajectoriesTogether(Autos.TWO_TO_B.getTrajectory()),
            Commands.sequence(
                new WaitCommand(1), AutoCommandGroups.startIntake(waitForFourbarDuringIntaking))),
        AutoCommandGroups.stopIntake(),
        Commands.sequence(
            SwerveSubsystem.Commands.stringTrajectoriesTogether(Autos.B_TO_TWO.getTrajectory()),
            AutoCommandGroups.prepScore(
                SuperstructureConstants.SCORE_CUBE_MID, waitForFourbarDuringScoring)),
        AutoCommandGroups.score(
            SuperstructureConstants.SCORE_CUBE_MID, delayAfterScoring, waitForFourbarDuringScoring),
        AutoCommandGroups.stopIntake().repeatedly().until(() -> Robot.fourBar.isAtTarget()),
        Elevator.Commands.setToHeightAndWait(0));
  }
}
