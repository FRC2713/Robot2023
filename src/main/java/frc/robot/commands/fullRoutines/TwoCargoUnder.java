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
import frc.robot.util.AutoPath;
import frc.robot.util.AutoPath.Autos;
import frc.robot.util.TrajectoryController;

public class TwoCargoUnder extends SequentialCommandGroup {
  public TwoCargoUnder() {
    addCommands(
        new InstantCommand(
            () ->
                Robot.swerveDrive.resetOdometry(
                    AutoPath.Autos.NINE_TO_D.getTrajectory().getInitialHolonomicPose())),
        FourBar.Commands.retract(),
        Elevator.Commands.elevatorConeHighScoreAndWait(),
        FourBar.Commands.extend(),
        Intake.Commands.setRollerVelocityRPM(100),
        Intake.Commands.setWheelVelocityRPM(100),
        new WaitCommand(1),
        FourBar.Commands.retract(),
        Intake.Commands.setRollerVelocityRPM(0),
        Intake.Commands.setWheelVelocityRPM(0),
        Elevator.Commands.elevatorConeFloorUpIntakeAndWait(),
        SwerveSubsystem.Commands.stringTrajectoriesTogether(Autos.NINE_TO_D.getTrajectory()),
        new WaitUntilCommand(() -> TrajectoryController.getInstance().isFinished()),
        FourBar.Commands.extend(),
        Intake.Commands.setRollerVelocityRPM(-100),
        Intake.Commands.setWheelVelocityRPM(-100),
        new WaitCommand(1),
        FourBar.Commands.retract(),
        Intake.Commands.setRollerVelocityRPM(0),
        Intake.Commands.setWheelVelocityRPM(0),
        SwerveSubsystem.Commands.stringTrajectoriesTogether(Autos.D_TO_SEVEN.getTrajectory()),
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
