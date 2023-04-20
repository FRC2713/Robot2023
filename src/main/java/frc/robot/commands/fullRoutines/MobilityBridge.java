package frc.robot.commands.fullRoutines;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.Robot;
import frc.robot.Robot.GamePieceMode;
import frc.robot.commands.PIDOnBridgeExperimental;
import frc.robot.subsystems.elevatorIO.Elevator;
import frc.robot.subsystems.fourBarIO.FourBar;
import frc.robot.subsystems.intakeIO.Intake;
import frc.robot.subsystems.swerveIO.SwerveSubsystem;
import frc.robot.util.AutoPath;
import frc.robot.util.SuperstructureConfig;

public class MobilityBridge extends SequentialCommandGroup {

  private double delayAfterScoring = 0.5;
  private boolean waitForFourbarDuringScoring = true;
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
  // AutoPath.Autos.TRAJ_MOBILITY.getTrajectory())
  //             //       ,
  //             //   Commands.sequence(Commands.waitSeconds(0.5), Slapper.Commands.comeBackHome())
  //             ),
  //         new PIDOnBridgeExperimental(false));
  //   }
  public Command initializeOdometry(Pose2d initialPose, double allianceOffsetMeters) {
    return new InstantCommand(
        () ->
            Robot.swerveDrive.resetOdometry(
                initialPose.plus(
                    DriverStation.getAlliance() == Alliance.Blue
                        ? new Transform2d(
                            new Translation2d(0, -allianceOffsetMeters), new Rotation2d())
                        : new Transform2d(
                            new Translation2d(0, allianceOffsetMeters), new Rotation2d()))));
  }

  public Command prepScore(SuperstructureConfig config, boolean waitForFourbar) {
    return Commands.sequence(
        Elevator.Commands.setToHeightAndWait(config),
        waitForFourbar
            ? FourBar.Commands.setAngleDegAndWait(config)
            : FourBar.Commands.setToAngle(config));
  }

  public Command score(SuperstructureConfig config, double waitAtEnd, boolean waitForFourbar) {
    return Commands.sequence(
        new InstantCommand(() -> Robot.intake.setScoring(true)),
        prepScore(config, waitForFourbar),
        Intake.Commands.score(),
        new WaitCommand(waitAtEnd));
  }

  public static Command stopIntake() {
    SuperstructureConfig holdConfig =
        Robot.gamePieceMode == GamePieceMode.CUBE
            ? SuperstructureConstants.HOLD_CUBE
            : SuperstructureConstants.HOLD_CONE;

    return new ParallelCommandGroup(
        new InstantCommand(() -> Robot.intake.setScoring(false)),
        Intake.Commands.setBottomVelocityRPM(holdConfig.getBottomRPM()),
        Intake.Commands.setTopVelocityRPM(holdConfig.getTopRPM()),
        FourBar.Commands.retract());
  }

  public MobilityBridge() {
    addCommands(
        initializeOdometry(
            AutoPath.Autos.NO_SLAP_TRAJ_MOBILITY.getTrajectory().getInitialHolonomicPose(), 0),

        // Score preload

        // new InstantCommand(
        //     () -> {
        //       Robot.gamePieceMode = GamePieceMode.CONE;
        //     }),
        // Intake.Commands.setBottomVelocityRPM(SuperstructureConstants.HOLD_CONE.getBottomRPM()),
        // Intake.Commands.setTopVelocityRPM(SuperstructureConstants.HOLD_CONE.getTopRPM()),
        // score(
        //     SuperstructureConstants.SCORE_CONE_HIGH,
        //     delayAfterScoring,
        //     waitForFourbarDuringScoring),
        // // Slapper.Commands.sendIt(),
        // stopIntake().repeatedly().until(() -> Robot.fourBar.isAtTarget()),


        // new WaitCommand(0.5),

        // new ParallelCommandGroup(null)

        Commands.parallel(
            // SwerveSubsystem.Commands.stringTrajectoriesTogether(
            //     AutoPath.Autos.NO_SLAP_TRAJ_MOBILITY.getTrajectory()),

            // Score preload
            // Commands.sequence(new WaitCommand(0.5), Elevator.Commands.setToHeight(0))

            //       ,
            //   Commands.sequence(Commands.waitSeconds(0.5), Slapper.Commands.comeBackHome())
            ),
        new PIDOnBridgeExperimental(false));
  }
}
