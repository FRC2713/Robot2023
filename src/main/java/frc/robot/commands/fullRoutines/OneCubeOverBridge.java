package frc.robot.commands.fullRoutines;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.Robot;
import frc.robot.Robot.GamePieceMode;
import frc.robot.commands.GetOnBridge;
import frc.robot.subsystems.elevatorIO.Elevator;
import frc.robot.subsystems.swerveIO.SwerveSubsystem;
import frc.robot.util.AutoPath.Autos;

public class OneCubeOverBridge extends SequentialCommandGroup {

  public OneCubeOverBridge() {
    addCommands(
        // Score preload
        new InstantCommand(
            () -> {
              Robot.swerveDrive.resetOdometry(
                  Autos.TWO_TO_A.getTrajectory().getInitialHolonomicPose());
              Robot.gamePieceMode = GamePieceMode.CUBE;
              Robot.intake.setScoring(true);
              Robot.fourBar.reseed();
            }),
        Commands.parallel(
            AutoCommandGroups.scoreLowNoPrep(),
            Elevator.Commands.setToHeightAndWait(SuperstructureConstants.INTAKE_CUBE),
            SwerveSubsystem.Commands.stringTrajectoriesTogether(Autos.TWO_TO_A.getTrajectory())),
        AutoCommandGroups.stopIntake(),

        // Move to Charge Station
        SwerveSubsystem.Commands.stringTrajectoriesTogether(Autos.A_TO_BRIDGE.getTrajectory()),
        new GetOnBridge(false));
  }
}
