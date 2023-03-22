package frc.robot.commands.fullRoutines;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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

public class ScoreCommunityUnder extends SequentialCommandGroup {

  // score based on where it goes
  private Command score(SuperstructureConfig config) {
    return Commands.sequence(
        new InstantCommand(() -> Robot.intake.setScoring(true)),
        prepScore(config),
        Intake.Commands.score(),
        new WaitCommand(0.5));
  }

  // prepare to score
  private Command prepScore(SuperstructureConfig config) {
    return Commands.sequence(
        Elevator.Commands.setToHeightAndWait(config),
        FourBar.Commands.setToAngle(config.getFourBarPosition()));
  }

  // intake turn on
  private Command startIntake() {
    return new ConditionalCommand(
        new ParallelCommandGroup(
            new InstantCommand(() -> Robot.intake.setScoring(false)),
            FourBar.Commands.setToAngle(
                Constants.SuperstructureConstants.INTAKE_CUBE.getFourBarPosition()),
            Intake.Commands.setBottomVelocityRPM(
                Constants.SuperstructureConstants.INTAKE_CUBE.getBottomRPM()),
            Intake.Commands.setTopVelocityRPM(
                Constants.SuperstructureConstants.INTAKE_CUBE.getTopRPM())),
        new ParallelCommandGroup(
            new InstantCommand(() -> Robot.intake.setScoring(false)),
            FourBar.Commands.setToAngle(
                Constants.SuperstructureConstants.INTAKE_TIPPED_CONE.getFourBarPosition()),
            Intake.Commands.setBottomVelocityRPM(
                Constants.SuperstructureConstants.INTAKE_TIPPED_CONE.getBottomRPM()),
            Intake.Commands.setTopVelocityRPM(
                Constants.SuperstructureConstants.INTAKE_TIPPED_CONE.getTopRPM())),
        () -> Robot.gamePieceMode == GamePieceMode.CUBE);
  }

  // intake but spitting things out, subject to change
  private Command spitIntake() {
    return new ParallelCommandGroup(
        new InstantCommand(() -> Robot.intake.setScoring(true)),
        FourBar.Commands.setToAngle(
            Constants.SuperstructureConstants.INTAKE_CUBE.getFourBarPosition()),
        Intake.Commands.setBottomVelocityRPM(
            Constants.SuperstructureConstants.SCORE_CUBE_HIGH.getBottomRPM() * 1.5),
        Intake.Commands.setTopVelocityRPM(
            Constants.SuperstructureConstants.SCORE_CUBE_HIGH.getTopRPM() * 1.5));
  }

  private Command stopIntake() {
    return new ConditionalCommand(
        new ParallelCommandGroup(
            new InstantCommand(() -> Robot.intake.setScoring(false)),
            Intake.Commands.setBottomVelocityRPM(SuperstructureConstants.HOLD_CUBE.getBottomRPM()),
            Intake.Commands.setTopVelocityRPM(SuperstructureConstants.HOLD_CUBE.getTopRPM()),
            FourBar.Commands.retract()),
        new ParallelCommandGroup(
            new InstantCommand(() -> Robot.intake.setScoring(false)),
            Intake.Commands.setBottomVelocityRPM(SuperstructureConstants.HOLD_CONE.getBottomRPM()),
            Intake.Commands.setTopVelocityRPM(SuperstructureConstants.HOLD_CONE.getTopRPM()),
            FourBar.Commands.retract()),
        () -> Robot.gamePieceMode == GamePieceMode.CUBE);
  }

  public ScoreCommunityUnder() {
    addCommands(
        new InstantCommand(
            () -> {
              Robot.swerveDrive.resetOdometry(
                  Autos.EIGHT_TO_D.getTrajectory().getInitialHolonomicPose());
              Robot.gamePieceMode = GamePieceMode.CUBE;
              Robot.intake.setScoring(true);
            }),
        Commands.sequence(
            prepScore(SuperstructureConstants.SCORE_CUBE_HIGH),
            score(SuperstructureConstants.SCORE_CUBE_HIGH)),
        Commands.parallel(
            SwerveSubsystem.Commands.stringTrajectoriesTogether(Autos.EIGHT_TO_D.getTrajectory()),
            Commands.sequence(
                new WaitCommand(0.5),
                Elevator.Commands.setToHeight(SuperstructureConstants.INTAKE_CUBE)),
            startIntake()),
        SwerveSubsystem.Commands.stringTrajectoriesTogether(Autos.D_TO_COMMUNITY.getTrajectory()),
        spitIntake(),
        new WaitCommand(0.5),
        startIntake(),
        SwerveSubsystem.Commands.stringTrajectoriesTogether(Autos.COMMUNITY_TO_C.getTrajectory()),
        SwerveSubsystem.Commands.stringTrajectoriesTogether(Autos.C_TO_COMMUNITY.getTrajectory()),
        spitIntake(),
        new WaitCommand(0.5),
        stopIntake());
  }
}
