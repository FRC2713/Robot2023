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

public class TwoCubeOver extends SequentialCommandGroup {

  private Command score(SuperstructureConfig config) {
    return Commands.sequence(prepScore(config), Intake.Commands.score(), new WaitCommand(0.5));
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
                Constants.SuperstructureConstants.INTAKE_UPRIGHT_CONE.getFourBarPosition()),
            Intake.Commands.setBottomVelocityRPM(
                Constants.SuperstructureConstants.INTAKE_UPRIGHT_CONE.getBottomRPM()),
            Intake.Commands.setTopVelocityRPM(
                Constants.SuperstructureConstants.INTAKE_UPRIGHT_CONE.getTopRPM())),
        () -> Robot.gamePieceMode == GamePieceMode.CUBE);
  }

  private Command stopIntake() {
    return new ConditionalCommand(
        new ParallelCommandGroup(
            Intake.Commands.setBottomVelocityRPM(0),
            Intake.Commands.setTopVelocityRPM(0),
            FourBar.Commands.retract()),
        new ParallelCommandGroup(
            Intake.Commands.setBottomVelocityRPM(-500),
            Intake.Commands.setTopVelocityRPM(-500),
            FourBar.Commands.retract()),
        () -> Robot.gamePieceMode == GamePieceMode.CUBE);
  }

  public TwoCubeOver() {
    addCommands(
        new InstantCommand(
            () -> {
              //   Robot.swerveDrive.resetGyro(
              //       Autos.TWO_TO_A.getTrajectory().getInitialHolonomicPose().getRotation());
              Robot.swerveDrive.resetOdometry(
                  Autos.TWO_TO_A.getTrajectory().getInitialHolonomicPose());
              Robot.gamePieceMode = GamePieceMode.CUBE;
            }),
        Commands.parallel(
            Commands.sequence(Intake.Commands.score(), new WaitCommand(0.5), startIntake()),
            Elevator.Commands.setToHeightAndWait(SuperstructureConstants.INTAKE_CUBE),
            SwerveSubsystem.Commands.stringTrajectoriesTogether(Autos.TWO_TO_A.getTrajectory())),
        new WaitUntilCommand(() -> TrajectoryController.getInstance().isFinished()),
        stopIntake(),
        Commands.parallel(
            SwerveSubsystem.Commands.stringTrajectoriesTogether(Autos.A_TO_TWO.getTrajectory()),
            prepScore(SuperstructureConstants.SCORE_CUBE_MID)),
        new WaitUntilCommand(() -> TrajectoryController.getInstance().isFinished()),
        score(SuperstructureConstants.SCORE_CUBE_MID),
        stopIntake(),
        Elevator.Commands.setToHeight(Constants.zero));
  }
}
