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

public class ScoreCommunityUnder extends SequentialCommandGroup {

  private double delayAfterScoring = 0.5;
  private boolean waitForFourbarDuringScoring = false;
  private boolean waitForFourbarDuringIntaking = false;

  public ScoreCommunityUnder() {
    addCommands(
        // Score preload
        new InstantCommand(
            () -> {
              Robot.swerveDrive.resetOdometry(
                  Autos.EIGHT_TO_D.getTrajectory().getInitialHolonomicPose());
              Robot.gamePieceMode = GamePieceMode.CUBE;
              Robot.intake.setScoring(true);
              Robot.fourBar.reseed();
            }),
        Commands.sequence(
            AutoCommandGroups.prepScore(
                SuperstructureConstants.SCORE_CUBE_HIGH, waitForFourbarDuringScoring),
            AutoCommandGroups.score(
                SuperstructureConstants.SCORE_CUBE_HIGH,
                delayAfterScoring,
                waitForFourbarDuringScoring)),

        // Obtain and spit out a cube towards the grid
        Commands.parallel(
            SwerveSubsystem.Commands.stringTrajectoriesTogether(Autos.EIGHT_TO_D.getTrajectory()),
            Commands.sequence(
                new WaitCommand(0.5),
                Elevator.Commands.setToHeight(SuperstructureConstants.INTAKE_CUBE)),
            AutoCommandGroups.startIntake(waitForFourbarDuringIntaking)),
        SwerveSubsystem.Commands.stringTrajectoriesTogether(Autos.D_TO_COMMUNITY.getTrajectory()),
        AutoCommandGroups.spitCube(),
        new WaitCommand(0.5),
        AutoCommandGroups.startIntake(waitForFourbarDuringIntaking),
        SwerveSubsystem.Commands.stringTrajectoriesTogether(Autos.COMMUNITY_TO_C.getTrajectory()),
        SwerveSubsystem.Commands.stringTrajectoriesTogether(Autos.C_TO_COMMUNITY.getTrajectory()),
        AutoCommandGroups.spitCube(),
        new WaitCommand(0.5),
        AutoCommandGroups.stopIntake());
  }
}
