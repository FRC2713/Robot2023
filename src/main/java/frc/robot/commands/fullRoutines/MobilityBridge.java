package frc.robot.commands.fullRoutines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class MobilityBridge extends SequentialCommandGroup {
  //   public Command initializeOdometry(Pose2d initialPose, double allianceOffsetMeters) {
  //     return new InstantCommand(
  //         () ->
  //             Robot.swerveDrive.resetOdometry(
  //                 initialPose.plus(
  //                     DriverStation.getAlliance() == Alliance.Blue
  //                         ? new Transform2d(
  //                             new Translation2d(0, -allianceOffsetMeters), new Rotation2d())
  //                         : new Transform2d(
  //                             new Translation2d(0, allianceOffsetMeters), new Rotation2d()))));
  //   }

  //   public MobilityBridge() {
  //     addCommands(
  //         initializeOdometry(
  //             AutoPath.Autos.TRAJ_MOBILITY.getTrajectory().getInitialHolonomicPose(), 0.3),
  //         // Slapper.Commands.sendIt(),
  //         Commands.parallel(
  //             SwerveSubsystem.Commands.stringTrajectoriesTogether(
  //                 AutoPath.Autos.TRAJ_MOBILITY.getTrajectory())
  //             //       ,
  //             //   Commands.sequence(Commands.waitSeconds(0.5), Slapper.Commands.comeBackHome())
  //             ),
  //         new PIDOnBridgeExperimental(false));
  //   }
}
