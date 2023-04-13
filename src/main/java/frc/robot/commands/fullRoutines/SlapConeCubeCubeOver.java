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
import frc.robot.subsystems.slapperIO.Slapper;
import frc.robot.subsystems.swerveIO.SwerveSubsystem;
import frc.robot.util.AutoPath.Autos;
import frc.robot.util.SuperstructureConfig;

public class SlapConeCubeCubeOver extends SequentialCommandGroup {
  private Command score(SuperstructureConfig config) {
    return Commands.sequence(
        new InstantCommand(() -> Robot.intake.setScoring(true)),
        prepScore(config),
        Intake.Commands.score(),
        new WaitCommand(0.5));
  }

  private Command prepScore(SuperstructureConfig config) {
    return Commands.sequence(
        Elevator.Commands.setToHeightAndWait(config), FourBar.Commands.setToAngle(config));
  }

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

  public SlapConeCubeCubeOver() {
    addCommands(
        new InstantCommand(
            () -> {
              Robot.swerveDrive.resetOdometry(
                  Autos.SLAP_ONE_TO_A.getTrajectory().getInitialHolonomicPose());
            }),
        Slapper.Commands.sendItAndWait(),
        new InstantCommand(() -> Robot.gamePieceMode = GamePieceMode.CUBE),
        Commands.parallel(
            SwerveSubsystem.Commands.stringTrajectoriesTogether(
                Autos.SLAP_ONE_TO_A.getTrajectory()),
            Commands.sequence(
                new WaitCommand(0.5),
                Elevator.Commands.setToHeight(SuperstructureConstants.INTAKE_CUBE),
                startIntake(),
                new WaitCommand(0.5),
                Slapper.Commands.comeBackHome())),
        stopIntake(),
        Commands.parallel(
            prepScore(SuperstructureConstants.SCORE_CUBE_HIGH),
            SwerveSubsystem.Commands.stringTrajectoriesTogether(Autos.A_TO_TWO.getTrajectory())),
        score(SuperstructureConstants.SCORE_CUBE_HIGH),
        stopIntake(),
        new InstantCommand(() -> Robot.gamePieceMode = GamePieceMode.CUBE),
        Commands.parallel(
            // Go to second cube
            SwerveSubsystem.Commands.stringTrajectoriesTogether(Autos.TWO_TO_B.getTrajectory()),
            Commands.sequence(
                new WaitCommand(0.5),
                Elevator.Commands.setToHeight(SuperstructureConstants.INTAKE_CUBE),
                startIntake())),
        Commands.parallel(
            prepScore(SuperstructureConstants.SCORE_CUBE_HIGH),
            SwerveSubsystem.Commands.stringTrajectoriesTogether(Autos.B_TO_FIVE.getTrajectory())),
        score(SuperstructureConstants.SCORE_CUBE_HIGH),
        stopIntake(),
        Elevator.Commands.setToHeight(0));
    ;
  }
}
