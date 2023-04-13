package frc.robot.commands.fullRoutines;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.Robot;
import frc.robot.Robot.GamePieceMode;
import frc.robot.subsystems.elevatorIO.Elevator;
import frc.robot.subsystems.swerveIO.SwerveSubsystem;
import frc.robot.util.AutoPath.Autos;

public class FastThreeCubeOver extends SequentialCommandGroup {

  private boolean waitForFourbarDuringScoring = true;

  public FastThreeCubeOver() {
    addCommands(
        new InstantCommand(
            () -> {
              Robot.swerveDrive.resetOdometry(
                  Autos.ONE_TO_A.getTrajectory().getInitialHolonomicPose());
              Robot.gamePieceMode = GamePieceMode.CUBE;
              Robot.intake.setScoring(true);
              Robot.fourBar.reseed();
            }),
        Commands.parallel(
            AutoCommandGroups.scoreLowNoPrep(),
            Elevator.Commands.setToHeight(SuperstructureConstants.INTAKE_CUBE),
            SwerveSubsystem.Commands.stringTrajectoriesTogether(Autos.ONE_TO_A.getTrajectory())),
        AutoCommandGroups.stopIntake(),
        Commands.parallel(
            SwerveSubsystem.Commands.stringTrajectoriesTogether(Autos.A_TO_TWO.getTrajectory()),
            AutoCommandGroups.prepScore(
                SuperstructureConstants.SCORE_CUBE_LOW, waitForFourbarDuringScoring)),
        Commands.parallel(
            Elevator.Commands.setToHeightAndWait(SuperstructureConstants.INTAKE_CUBE),
            AutoCommandGroups.scoreLowNoPrep(),
            SwerveSubsystem.Commands.stringTrajectoriesTogether(Autos.TWO_TO_B.getTrajectory())),
        AutoCommandGroups.stopIntake(),
        Commands.parallel(
            SwerveSubsystem.Commands.stringTrajectoriesTogether(Autos.B_TO_THREE.getTrajectory()),
            AutoCommandGroups.prepScore(
                SuperstructureConstants.SCORE_CUBE_LOW, waitForFourbarDuringScoring)),
        AutoCommandGroups.scoreLowNoPrep(),
        AutoCommandGroups.stopIntake().repeatedly().until(() -> Robot.fourBar.isAtTarget()),
        Elevator.Commands.setToHeightAndWait(Constants.zero));
  }
}
