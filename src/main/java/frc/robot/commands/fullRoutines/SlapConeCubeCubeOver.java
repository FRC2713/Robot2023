package frc.robot.commands.fullRoutines;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.Robot;
import frc.robot.Robot.GamePieceMode;
import frc.robot.subsystems.elevatorIO.Elevator;
import frc.robot.subsystems.slapperIO.Slapper;
import frc.robot.subsystems.swerveIO.SwerveSubsystem;
import frc.robot.util.AutoPath.Autos;

public class SlapConeCubeCubeOver extends SequentialCommandGroup {

  private double delayAfterScoring = 0.5;
  private boolean waitForFourbarDuringScoring = true;
  private boolean waitForFourbarDuringIntaking = false;

  public SlapConeCubeCubeOver() {
    addCommands(
        AutoCommandGroups.initializeOdometry(
            Autos.SLAP_ONE_TO_A.getTrajectory().getInitialHolonomicPose(), 0.3),
        Slapper.Commands.sendItAndWait(),
        new InstantCommand(() -> Robot.gamePieceMode = GamePieceMode.CUBE),
        Commands.parallel(
            SwerveSubsystem.Commands.stringTrajectoriesTogether(
                Autos.SLAP_ONE_TO_A.getTrajectory()),
            Commands.sequence(
                new WaitCommand(0.5),
                Elevator.Commands.setToHeight(SuperstructureConstants.INTAKE_CUBE),
                AutoCommandGroups.startIntake(waitForFourbarDuringIntaking),
                Slapper.Commands.comeBackHome())),
        AutoCommandGroups.stopIntake(),
        SwerveSubsystem.Commands.stringTrajectoriesTogether(Autos.A_TO_TWO.getTrajectory()),
        AutoCommandGroups.score(
            SuperstructureConstants.SCORE_CUBE_HIGH,
            delayAfterScoring,
            waitForFourbarDuringScoring),
        AutoCommandGroups.stopIntake(),
        Commands.parallel(
            SwerveSubsystem.Commands.stringTrajectoriesTogether(Autos.TWO_TO_B.getTrajectory()),
            Commands.sequence(
                new WaitCommand(0.5),
                Elevator.Commands.setToHeight(SuperstructureConstants.INTAKE_CUBE),
                AutoCommandGroups.startIntake(waitForFourbarDuringIntaking))),
        SwerveSubsystem.Commands.stringTrajectoriesTogether(Autos.B_TO_TWO.getTrajectory()),
        AutoCommandGroups.score(
            SuperstructureConstants.SCORE_CUBE_MID, delayAfterScoring, waitForFourbarDuringScoring),
        AutoCommandGroups.stopIntake(),
        Elevator.Commands.setToHeight(0));
    ;
  }
}
