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
import frc.robot.util.AutoPath;
import frc.robot.util.AutoPath.Autos;
import frc.robot.util.SuperstructureConfig;

public class OneConeOneCubeUnder extends SequentialCommandGroup {

  private Command score(SuperstructureConfig config) {
    return Commands.sequence(
        new InstantCommand(() -> Robot.intake.setScoring(true)),
        prepScore(config),
        Intake.Commands.score(),
        new WaitCommand(0.5));
  }

  private Command prepScore(SuperstructureConfig config) {
    return Commands.sequence(
        Elevator.Commands.setToHeightAndWait(config),
        FourBar.Commands.setAngleDegAndWait(config.getFourBarPosition()));
  }

  private Command startIntake() {
    return new ConditionalCommand(
        new ParallelCommandGroup(
            new InstantCommand(() -> Robot.intake.setScoring(false)),
            FourBar.Commands.setAngleDegAndWait(
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

  public OneConeOneCubeUnder() {
    addCommands(
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
        score(SuperstructureConstants.SCORE_CONE_HIGH),
        stopIntake(),
        new InstantCommand(() -> Robot.gamePieceMode = GamePieceMode.CUBE),
        Elevator.Commands.setToHeightAndWait(SuperstructureConstants.INTAKE_CUBE),
        startIntake(),
        SwerveSubsystem.Commands.stringTrajectoriesTogether(Autos.NINE_TO_D.getTrajectory()),
        stopIntake(),
        Commands.parallel(
            prepScore(SuperstructureConstants.SCORE_CUBE_LOW),
            SwerveSubsystem.Commands.stringTrajectoriesTogether(Autos.D_TO_EIGHT.getTrajectory())),
        score(SuperstructureConstants.SCORE_CUBE_LOW),
        stopIntake(),
        Elevator.Commands.setToHeightAndWait(Constants.zero));
  }
}
