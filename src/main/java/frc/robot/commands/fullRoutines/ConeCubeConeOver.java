package frc.robot.commands.fullRoutines;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.Robot;
import frc.robot.Robot.GamePieceMode;
import frc.robot.subsystems.elevatorIO.Elevator;
import frc.robot.subsystems.fourBarIO.FourBar;
import frc.robot.subsystems.intakeIO.Intake;
import frc.robot.subsystems.swerveIO.SwerveSubsystem;
import frc.robot.util.AutoPath.Autos;
import frc.robot.util.SuperstructureConfig;
import frc.robot.util.TrajectoryController;

public class ConeCubeConeOver extends SequentialCommandGroup {

  private Command score(SuperstructureConfig config) {
    return Commands.sequence(
        new InstantCommand(() -> Robot.intake.setScoring(true)),
        prepScore(config),
        Intake.Commands.score(),
        new WaitCommand(1));
  }

  private Command prepScore(SuperstructureConfig config) {
    return Commands.sequence(
        Elevator.Commands.setToHeightAndWait(config),
        FourBar.Commands.setToAngle(config.getFourBarPosition()));
  }

  private Command startIntake() {
    return new ConditionalCommand(
        new ParallelCommandGroup(
            FourBar.Commands.setToAngle(
                Constants.SuperstructureConstants.INTAKE_CUBE.getFourBarPosition()),
            Intake.Commands.setBottomVelocityRPM(
                Constants.SuperstructureConstants.INTAKE_CUBE.getBottomRPM()),
            Intake.Commands.setTopVelocityRPM(
                Constants.SuperstructureConstants.INTAKE_CUBE.getTopRPM())),
        new ParallelCommandGroup(
            FourBar.Commands.setToAngle(
                Constants.SuperstructureConstants.INTAKE_TIPPED_CONE.getFourBarPosition()),
            Intake.Commands.setBottomVelocityRPM(
                Constants.SuperstructureConstants.INTAKE_TIPPED_CONE.getBottomRPM()),
            Intake.Commands.setTopVelocityRPM(
                Constants.SuperstructureConstants.INTAKE_TIPPED_CONE.getTopRPM())),
        () -> Robot.gamePieceMode == GamePieceMode.CUBE);
  }

  private Command stopIntake() {
    return new ConditionalCommand(
        new ParallelCommandGroup(
            new InstantCommand(() -> Robot.intake.setScoring(false)),
            Intake.Commands.setBottomVelocityRPM(0),
            Intake.Commands.setTopVelocityRPM(0),
            FourBar.Commands.retract()),
        new ParallelCommandGroup(
            new InstantCommand(() -> Robot.intake.setScoring(false)),
            Intake.Commands.setBottomVelocityRPM(-500),
            Intake.Commands.setTopVelocityRPM(-500),
            FourBar.Commands.retract()),
        () -> Robot.gamePieceMode == GamePieceMode.CUBE);
  }

  public ConeCubeConeOver() {
    addCommands(
        new InstantCommand(
            () -> {
              Robot.swerveDrive.resetOdometry(
                  Autos.ONE_TO_A.getTrajectory().getInitialHolonomicPose());
              Robot.gamePieceMode = GamePieceMode.CONE;
            }),
        score(SuperstructureConstants.SCORE_CONE_HIGH),
        Commands.parallel(
            SwerveSubsystem.Commands.stringTrajectoriesTogether(Autos.ONE_TO_A.getTrajectory()),
            Commands.sequence(
                new WaitCommand(0.5),
                new InstantCommand(() -> Robot.gamePieceMode = GamePieceMode.CUBE),
                new InstantCommand(() -> Robot.intake.setScoring(false)),
                startIntake(),
                Elevator.Commands.setToHeightAndWait(SuperstructureConstants.INTAKE_CUBE))),
        // new InstantCommand(() -> Robot.gamePieceMode = GamePieceMode.CUBE),
        // startIntake(),
        new WaitUntilCommand(() -> TrajectoryController.getInstance().isFinished()),
        stopIntake(),
        Commands.parallel(
            SwerveSubsystem.Commands.stringTrajectoriesTogether(Autos.A_TO_TWO.getTrajectory()),
            Commands.sequence(
                new WaitCommand(1),
                prepScore(SuperstructureConstants.SCORE_CUBE_MID),
                stopIntake())),
        new WaitUntilCommand(() -> TrajectoryController.getInstance().isFinished()),
        score(SuperstructureConstants.SCORE_CUBE_HIGH),
        stopIntake().repeatedly().until(() -> Robot.fourBar.isAtTarget()),
        Commands.parallel(
            Commands.sequence(
                new InstantCommand(() -> Robot.gamePieceMode = GamePieceMode.CONE),
                new InstantCommand(() -> Robot.intake.setScoring(false)),
                Elevator.Commands.setToHeightAndWait(SuperstructureConstants.INTAKE_TIPPED_CONE),
                startIntake()),
            SwerveSubsystem.Commands.stringTrajectoriesTogether(Autos.TWO_TO_B.getTrajectory())),
        new WaitUntilCommand(() -> TrajectoryController.getInstance().isFinished()),
        stopIntake(),
        Commands.parallel(
            SwerveSubsystem.Commands.stringTrajectoriesTogether(Autos.B_TO_THREE.getTrajectory()),
            prepScore(SuperstructureConstants.SCORE_CONE_MID)),
        new WaitUntilCommand(() -> TrajectoryController.getInstance().isFinished()),
        score(SuperstructureConstants.SCORE_CONE_MID),
        stopIntake().repeatedly().until(() -> Robot.fourBar.isAtTarget()),
        Elevator.Commands.setToHeightAndWait(0));
  }
}
