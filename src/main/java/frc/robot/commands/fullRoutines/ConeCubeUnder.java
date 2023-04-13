package frc.robot.commands.fullRoutines;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.Robot;
import frc.robot.Robot.GamePieceMode;
import frc.robot.subsystems.elevatorIO.Elevator;
import frc.robot.subsystems.fourBarIO.FourBar;
import frc.robot.subsystems.intakeIO.Intake;
import frc.robot.subsystems.swerveIO.SwerveSubsystem;
import frc.robot.util.AutoPath;
import frc.robot.util.AutoPath.Autos;

public class ConeCubeUnder extends SequentialCommandGroup {

  private double delayAfterScoring = 0.5;
  private boolean waitForFourbarDuringScoring = true;
  private boolean waitForFourbarDuringIntaking = true;

  public ConeCubeUnder() {
    addCommands(
        // Score Preload
        new InstantCommand(
            () -> {
              Robot.swerveDrive.resetOdometry(
                  AutoPath.Autos.NINE_TO_D.getTrajectory().getInitialHolonomicPose());
              Robot.gamePieceMode = GamePieceMode.CONE;
              //   Robot.fourBar.reseed();
            }),
        Intake.Commands.setBottomVelocityRPM(SuperstructureConstants.HOLD_CONE.getBottomRPM()),
        Intake.Commands.setTopVelocityRPM(SuperstructureConstants.HOLD_CONE.getTopRPM()),
        FourBar.Commands.retract(),
        AutoCommandGroups.score(
            SuperstructureConstants.SCORE_CONE_HIGH,
            delayAfterScoring,
            waitForFourbarDuringScoring),
        AutoCommandGroups.stopIntake(),

        // Get and score cube
        new InstantCommand(() -> Robot.gamePieceMode = GamePieceMode.CUBE),
        Elevator.Commands.setToHeightAndWait(SuperstructureConstants.INTAKE_CUBE),
        AutoCommandGroups.startIntake(waitForFourbarDuringIntaking),
        SwerveSubsystem.Commands.stringTrajectoriesTogether(Autos.NINE_TO_D.getTrajectory()),
        AutoCommandGroups.stopIntake(),
        Commands.parallel(
            AutoCommandGroups.prepScore(
                SuperstructureConstants.SCORE_CUBE_LOW, waitForFourbarDuringScoring),
            SwerveSubsystem.Commands.stringTrajectoriesTogether(Autos.D_TO_EIGHT.getTrajectory())),
        AutoCommandGroups.score(
            SuperstructureConstants.SCORE_CUBE_LOW, delayAfterScoring, waitForFourbarDuringScoring),
        AutoCommandGroups.stopIntake(),
        Elevator.Commands.setToHeightAndWait(Constants.zero));
  }
}
