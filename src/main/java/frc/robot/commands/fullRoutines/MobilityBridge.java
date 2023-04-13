package frc.robot.commands.fullRoutines;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.PIDOnBridgeExperimental;
import frc.robot.subsystems.slapperIO.Slapper;
import frc.robot.subsystems.swerveIO.SwerveSubsystem;
import frc.robot.util.AutoPath;

public class MobilityBridge extends SequentialCommandGroup {

  public MobilityBridge() {
    addCommands(
        AutoCommandGroups.initializeOdometry(
            AutoPath.Autos.TRAJ_MOBILITY.getTrajectory().getInitialHolonomicPose(), 0.3),
        Slapper.Commands.sendItAndWait(),
        Commands.parallel(
            SwerveSubsystem.Commands.stringTrajectoriesTogether(
                AutoPath.Autos.TRAJ_MOBILITY.getTrajectory()),
            Commands.sequence(Commands.waitSeconds(5), Slapper.Commands.comeBackHome())),
        new PIDOnBridgeExperimental(false));
  }
}
