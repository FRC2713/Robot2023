package frc.robot.commands.fullRoutines;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.GetOnBridge;
import frc.robot.subsystems.elevatorIO.Elevator;
import frc.robot.subsystems.fourBarIO.FourBar;
import frc.robot.subsystems.intakeIO.Intake;
import frc.robot.subsystems.swerveIO.SwerveSubsystem;
import frc.robot.util.AutoPath.Autos;
import frc.robot.util.TrajectoryController;

public class TwoCargoOver extends SequentialCommandGroup {

  private SequentialCommandGroup score() {
    return new SequentialCommandGroup(
        FourBar.Commands.extend(),
        Intake.Commands.setRollerVelocityRPM(Constants.IntakeConstants.ROLLERS_CONE_SCORE_RPM),
        Intake.Commands.setWheelVelocityRPM(Constants.IntakeConstants.WHEELS_CONE_SCORE_RPM),
        new WaitCommand(1));
  }

  private SequentialCommandGroup intake() {
    return new SequentialCommandGroup(
        FourBar.Commands.extend(),
        Intake.Commands.setRollerVelocityRPM(
            Constants.IntakeConstants.ROLLERS_CONE_UPRIGHT_INTAKE_RPM),
        Intake.Commands.setWheelVelocityRPM(
            Constants.IntakeConstants.WHEELS_CONE_UPRIGHT_INTAKE_RPM));
  }

  public TwoCargoOver() {
    addCommands(
        new InstantCommand(
            () -> {
              Robot.swerveDrive.resetOdometry(
                  Autos.ONE_TO_A.getTrajectory().getInitialHolonomicPose());
            }),
        FourBar.Commands.retract(),
        Elevator.Commands.elevatorConeHighScoreAndWait(),
        score(),
        Elevator.Commands.elevatorConeFloorUpIntakeAndWait(),
        SwerveSubsystem.Commands.stringTrajectoriesTogether(Autos.ONE_TO_A.getTrajectory()),
        intake(),
        new WaitUntilCommand(() -> TrajectoryController.getInstance().isFinished()),
        FourBar.Commands.retract(),
        Intake.Commands.setRollerVelocityRPM(0),
        Intake.Commands.setWheelVelocityRPM(0),
        SwerveSubsystem.Commands.stringTrajectoriesTogether(Autos.A_TO_FIVE.getTrajectory()),
        new WaitUntilCommand(() -> TrajectoryController.getInstance().isFinished()),
        Elevator.Commands.elevatorConeHighScoreAndWait(),
        FourBar.Commands.extend(),
        Intake.Commands.setRollerVelocityRPM(100),
        Intake.Commands.setWheelVelocityRPM(100),
        new WaitCommand(1),
        FourBar.Commands.retract(),
        Elevator.Commands.setTargetHeightAndWait(Constants.zero),
        new GetOnBridge());
  }
}
