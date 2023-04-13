package frc.robot.commands.fullRoutines;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.Robot;
import frc.robot.subsystems.elevatorIO.Elevator;
import frc.robot.subsystems.swerveIO.SwerveSubsystem;
import frc.robot.util.AutoPath.Autos;

public class ThreeCubeOver extends SequentialCommandGroup {

  private double delayAfterScoring = 0.5;
  private boolean waitForFourbarDuringScoring = false;
  private boolean waitForFourbarDuringIntaking = false;

  public ThreeCubeOver() {
    addCommands(
        // Score the first two cubes
        new TwoCubeOver(),

        // Obtain and score the next two cubes
        Commands.parallel(
            Elevator.Commands.setToHeightAndWait(SuperstructureConstants.INTAKE_CUBE),
            AutoCommandGroups.startIntake(waitForFourbarDuringIntaking),
            SwerveSubsystem.Commands.stringTrajectoriesTogether(Autos.TWO_TO_B.getTrajectory())),
        AutoCommandGroups.stopIntake(),
        Commands.parallel(
            SwerveSubsystem.Commands.stringTrajectoriesTogether(Autos.B_TO_TWO.getTrajectory()),
            Commands.sequence(
                new WaitCommand(0.5),
                AutoCommandGroups.prepScore(
                    SuperstructureConstants.SCORE_CUBE_HIGH, waitForFourbarDuringScoring))),
        AutoCommandGroups.score(
            SuperstructureConstants.SCORE_CUBE_HIGH,
            delayAfterScoring,
            waitForFourbarDuringScoring),
        AutoCommandGroups.stopIntake().repeatedly().until(() -> Robot.fourBar.isAtTarget()),
        Elevator.Commands.setToHeightAndWait(Constants.zero));
  }
}
