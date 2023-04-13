package frc.robot.commands.fullRoutines;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.Robot;
import frc.robot.Robot.GamePieceMode;
import frc.robot.subsystems.elevatorIO.Elevator;
import frc.robot.subsystems.swerveIO.SwerveSubsystem;
import frc.robot.util.AutoPath.Autos;

public class SlapConeCubeConeOver extends SequentialCommandGroup {

  private double delayAfterScoring = 0.5;
  private boolean waitForFourbarDuringScoring = false;
  private boolean waitForFourbarDuringIntaking = false;

  public SlapConeCubeConeOver() {
    addCommands(
        // new InstantCommand(
        //     () -> {
        //       Robot.swerveDrive.resetOdometry(
        //           Autos.REVERSE_ONE_TO_A.getTrajectory().getInitialHolonomicPose());
        //     }),
        // Slapper.Commands.sendItAndWait(),
        // new InstantCommand(() -> Robot.gamePieceMode = GamePieceMode.CUBE),
        // Commands.parallel(
        //     SwerveSubsystem.Commands.stringTrajectoriesTogether(
        //         Autos.REVERSE_ONE_TO_A.getTrajectory()),
        //     Commands.sequence(
        //         new WaitCommand(0.5),
        //         Elevator.Commands.setToHeight(SuperstructureConstants.INTAKE_CUBE),
        //         CommandGroups.startIntake(waitForFourbarDuringIntaking),
        //         new WaitCommand(0.5),
        //         Slapper.Commands.comeBackHome())),
        AutoCommandGroups.stopIntake(),
        Commands.parallel(
            AutoCommandGroups.prepScore(
                SuperstructureConstants.SCORE_CUBE_HIGH, waitForFourbarDuringScoring),
            SwerveSubsystem.Commands.stringTrajectoriesTogether(Autos.A_TO_TWO.getTrajectory())),
        AutoCommandGroups.score(
            SuperstructureConstants.SCORE_CUBE_HIGH,
            delayAfterScoring,
            waitForFourbarDuringScoring),
        AutoCommandGroups.stopIntake(),
        new InstantCommand(() -> Robot.gamePieceMode = GamePieceMode.CONE),
        Commands.parallel(
            SwerveSubsystem.Commands.stringTrajectoriesTogether(Autos.TWO_TO_B.getTrajectory()),
            Commands.sequence(
                new WaitCommand(0.5),
                Elevator.Commands.setToHeight(SuperstructureConstants.INTAKE_TIPPED_CONE),
                AutoCommandGroups.startIntake(waitForFourbarDuringIntaking))),
        Commands.parallel(
            AutoCommandGroups.prepScore(
                SuperstructureConstants.SCORE_CONE_LOW, waitForFourbarDuringScoring),
            SwerveSubsystem.Commands.stringTrajectoriesTogether(Autos.B_TO_THREE.getTrajectory())),
        AutoCommandGroups.score(
            SuperstructureConstants.SCORE_CONE_LOW, delayAfterScoring, waitForFourbarDuringScoring),
        AutoCommandGroups.stopIntake(),
        Elevator.Commands.setToHeight(0));
    ;
  }
}
