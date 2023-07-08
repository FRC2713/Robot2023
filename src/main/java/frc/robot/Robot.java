// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.GetOnBridge;
import frc.robot.commands.OTF.GoClosestGrid;
import frc.robot.commands.OTF.GoHumanPlayer;
import frc.robot.commands.OnBridgeUntilMovement;
import frc.robot.commands.PIDOnBridge;
import frc.robot.commands.fullRoutines.ConeCubeConeOver;
import frc.robot.commands.fullRoutines.FastThreeCubeOver;
import frc.robot.commands.fullRoutines.MobilityBridge;
import frc.robot.commands.fullRoutines.MobilityBridgePathing;
import frc.robot.commands.fullRoutines.OneConeBridge;
import frc.robot.commands.fullRoutines.OneConeOneCubeUnder;
import frc.robot.commands.fullRoutines.OneConeOneCubeUnderHigh;
import frc.robot.commands.fullRoutines.OneConeTwoCubeOver;
import frc.robot.commands.fullRoutines.OneCubeOverBridge;
import frc.robot.commands.fullRoutines.ScoreCommunityUnder;
import frc.robot.commands.fullRoutines.SimpleCone;
import frc.robot.commands.fullRoutines.ThreeCubeOver;
import frc.robot.commands.fullRoutines.TwoConeOver;
import frc.robot.commands.fullRoutines.TwoConeUnder;
import frc.robot.commands.fullRoutines.TwoCubeOver;
import frc.robot.commands.fullRoutines.TwoCubeOverBridge;
import frc.robot.subsystems.LightStrip;
import frc.robot.subsystems.elevatorIO.Elevator;
import frc.robot.subsystems.elevatorIO.ElevatorIOSim;
import frc.robot.subsystems.elevatorIO.ElevatorIOSparks;
import frc.robot.subsystems.fourBarIO.FourBar;
import frc.robot.subsystems.fourBarIO.FourBarIOSim;
import frc.robot.subsystems.fourBarIO.FourBarIOSparks;
import frc.robot.subsystems.intakeIO.Intake;
import frc.robot.subsystems.intakeIO.IntakeIOSim;
import frc.robot.subsystems.intakeIO.IntakeIOSparks;
import frc.robot.subsystems.swerveIO.SwerveIOPigeon2;
import frc.robot.subsystems.swerveIO.SwerveIOSim;
import frc.robot.subsystems.swerveIO.SwerveSubsystem;
import frc.robot.subsystems.swerveIO.module.SwerveModuleIOSim;
import frc.robot.subsystems.swerveIO.module.SwerveModuleIOSparkMAX;
import frc.robot.subsystems.visionIO.Vision;
import frc.robot.subsystems.visionIO.Vision.SnapshotMode;
import frc.robot.subsystems.visionIO.VisionIOSim;
import frc.robot.subsystems.visionIO.VisionLimelight;
import frc.robot.util.AutoPath;
import frc.robot.util.MechanismManager;
import frc.robot.util.MotionHandler.MotionMode;
import frc.robot.util.RedHawkUtil;
import java.io.File;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  public enum GamePieceMode {
    CONE,
    CUBE;
  }

  private static MechanismManager mechManager;
  public static MotionMode motionMode = MotionMode.FULL_DRIVE;
  public static FourBar fourBar;
  public static Elevator elevator;
  public static Intake intake;
  public static Vision vision;
  //   public static Slapper slapper;
  public static SwerveSubsystem swerveDrive;
  public GoClosestGrid goClosestGrid;
  public GoHumanPlayer goHumanPlayer;
  public static LightStrip lights;
  private Command autoCommand;
  public static GamePieceMode gamePieceMode = GamePieceMode.CUBE;
  private LinearFilter canUtilizationFilter = LinearFilter.singlePoleIIR(0.25, 0.02);

  public static final CommandXboxController driver =
      new CommandXboxController(Constants.RobotMap.DRIVER_PORT);
  public static final CommandXboxController operator =
      new CommandXboxController(Constants.RobotMap.OPERATOR_PORT);

  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Autonomous Routine");

  public static double[] poseValue;
  DoubleArraySubscriber frontVisionPose;
  DoubleArraySubscriber rearVisionPose;

  Alliance currentAlliance = Alliance.Invalid;
  DoubleArraySubscriber frontCamera2TagPose;
  DoubleArraySubscriber rearCamera2TagPose;

  @Override
  public void robotInit() {
    NetworkTable frontTable =
        NetworkTableInstance.getDefault().getTable(Vision.Limelights.FRONT.table);
    NetworkTable rearTable =
        NetworkTableInstance.getDefault().getTable(Vision.Limelights.REAR.table);
    frontVisionPose = frontTable.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[] {});
    frontCamera2TagPose =
        frontTable.getDoubleArrayTopic("targetpose_cameraspace").subscribe(new double[] {});
    rearVisionPose = rearTable.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[] {});
    rearCamera2TagPose =
        rearTable.getDoubleArrayTopic("targetpose_cameraspace").subscribe(new double[] {});
    Logger.getInstance().addDataReceiver(new NT4Publisher());
    Logger.getInstance().recordMetadata("GitRevision", Integer.toString(GVersion.GIT_REVISION));
    Logger.getInstance().recordMetadata("GitSHA", GVersion.GIT_SHA);
    Logger.getInstance().recordMetadata("GitDate", GVersion.GIT_DATE);
    Logger.getInstance().recordMetadata("GitBranch", GVersion.GIT_BRANCH);
    Logger.getInstance().recordMetadata("BuildDate", GVersion.BUILD_DATE);
    if (isReal()) {
      File sda1 = new File(Constants.Logging.sda1Dir);
      File sda2 = new File(Constants.Logging.sda2Dir);

      if (sda1.exists() && sda1.isDirectory()) {
        Logger.getInstance().addDataReceiver(new WPILOGWriter(Constants.Logging.sda1Dir));
        Logger.getInstance().recordOutput("isLoggingToUsb", true);
      } else {
        RedHawkUtil.ErrHandler.getInstance()
            .addError(
                "Cannot log to "
                    + Constants.Logging.sda1Dir
                    + ", trying "
                    + Constants.Logging.sda2Dir);
        if (sda2.exists() && sda2.isDirectory()) {
          Logger.getInstance().addDataReceiver(new WPILOGWriter(Constants.Logging.sda2Dir));
          Logger.getInstance().recordOutput("isLoggingToUsb", true);
        } else {
          RedHawkUtil.ErrHandler.getInstance()
              .addError("Cannot log to " + Constants.Logging.sda2Dir);
          Logger.getInstance().recordOutput("isLoggingToUsb", false);
        }
      }
    } else {
      Logger.getInstance().recordOutput("isLoggingToUsb", false);
    }

    Logger.getInstance().start();

    fourBar = new FourBar(isSimulation() ? new FourBarIOSim() : new FourBarIOSparks());
    elevator = new Elevator(isSimulation() ? new ElevatorIOSim() : new ElevatorIOSparks());
    intake = new Intake(isSimulation() ? new IntakeIOSim() : new IntakeIOSparks());
    vision =
        new Vision(
            isSimulation() ? new VisionIOSim() : new VisionLimelight("limelight"),
            isSimulation() ? new VisionIOSim() : new VisionLimelight("limelight-rear"));
    lights = new LightStrip();
    // slapper = new Slapper(true ? new SlapperIOSim() : new SlapperIOSparks());

    // fourBar = new FourBar(true ? new FourBarIOSim() : new FourBarIOSparks());
    // elevator = new Elevator(true ? new ElevatorIOSim() : new ElevatorIOSparks());
    // intake = new Intake(true ? new IntakeIOSim() : new IntakeIOSparks());
    // vision = new Vision(true ? new VisionIOSim() : new VisionLimelight());

    swerveDrive =
        isSimulation()
            // true
            ? new SwerveSubsystem(
                new SwerveIOSim(),
                new SwerveModuleIOSim(Constants.DriveConstants.FRONT_LEFT),
                new SwerveModuleIOSim(Constants.DriveConstants.FRONT_RIGHT),
                new SwerveModuleIOSim(Constants.DriveConstants.BACK_LEFT),
                new SwerveModuleIOSim(Constants.DriveConstants.BACK_RIGHT))
            : new SwerveSubsystem(
                new SwerveIOPigeon2(),
                new SwerveModuleIOSparkMAX(Constants.DriveConstants.FRONT_LEFT),
                new SwerveModuleIOSparkMAX(Constants.DriveConstants.FRONT_RIGHT),
                new SwerveModuleIOSparkMAX(Constants.DriveConstants.BACK_LEFT),
                new SwerveModuleIOSparkMAX(Constants.DriveConstants.BACK_RIGHT));

    // mechManager = new MechanismManager();
    // goClosestGrid = new GoClosestGrid();
    // goHumanPlayer = new GoHumanPlayer();

    // checkAlliance();
    // buildAutoChooser();

    // // Driver Controls
    // if (Constants.DEBUG_MODE == DebugMode.MATCH) {
    //   driver
    //       .povUp()
    //       .onTrue(
    //           new InstantCommand(
    //               () -> {
    //                 motionMode = MotionMode.HEADING_CONTROLLER;
    //                 SwerveHeadingController.getInstance().setSetpoint(Rotation2d.fromDegrees(0));
    //               }));

    //   driver
    //       .povLeft()
    //       .onTrue(
    //           new InstantCommand(
    //               () -> {
    //                 motionMode = MotionMode.HEADING_CONTROLLER;
    //
    // SwerveHeadingController.getInstance().setSetpoint(Rotation2d.fromDegrees(90));
    //               }));

    //   driver
    //       .povDown()
    //       .onTrue(
    //           new InstantCommand(
    //               () -> {
    //                 motionMode = MotionMode.HEADING_CONTROLLER;
    //
    // SwerveHeadingController.getInstance().setSetpoint(Rotation2d.fromDegrees(180));
    //               }));

    //   driver
    //       .povRight()
    //       .onTrue(
    //           new InstantCommand(
    //               () -> {
    //                 motionMode = MotionMode.HEADING_CONTROLLER;
    //
    // SwerveHeadingController.getInstance().setSetpoint(Rotation2d.fromDegrees(270));
    //               }));
    // } else if (Constants.DEBUG_MODE == DebugMode.TUNE_MODULES) {
    //   driver
    //       .povUp()
    //       .whileTrue(
    //           new InstantCommand(
    //               () -> {
    //                 motionMode = MotionMode.NULL;
    //                 swerveDrive.setModuleStates(
    //                     new SwerveModuleState[] {
    //                       new SwerveModuleState(
    //                           Units.feetToMeters(Constants.TUNE_MODULES_DRIVE_SPEED),
    //                           Rotation2d.fromDegrees(0)),
    //                       new SwerveModuleState(
    //                           Units.feetToMeters(Constants.TUNE_MODULES_DRIVE_SPEED),
    //                           Rotation2d.fromDegrees(0)),
    //                       new SwerveModuleState(
    //                           Units.feetToMeters(Constants.TUNE_MODULES_DRIVE_SPEED),
    //                           Rotation2d.fromDegrees(0)),
    //                       new SwerveModuleState(
    //                           Units.feetToMeters(Constants.TUNE_MODULES_DRIVE_SPEED),
    //                           Rotation2d.fromDegrees(0))
    //                     });
    //               },
    //               swerveDrive))
    //       .onFalse(
    //           new InstantCommand(
    //               () -> {
    //                 motionMode = MotionMode.FULL_DRIVE;
    //               },
    //               swerveDrive));
    //   driver
    //       .povDown()
    //       .whileTrue(
    //           new InstantCommand(
    //               () -> {
    //                 motionMode = MotionMode.NULL;
    //                 swerveDrive.setModuleStates(
    //                     new SwerveModuleState[] {
    //                       new SwerveModuleState(
    //                           Units.feetToMeters(-Constants.TUNE_MODULES_DRIVE_SPEED),
    //                           Rotation2d.fromDegrees(90)),
    //                       new SwerveModuleState(
    //                           Units.feetToMeters(-Constants.TUNE_MODULES_DRIVE_SPEED),
    //                           Rotation2d.fromDegrees(90)),
    //                       new SwerveModuleState(
    //                           Units.feetToMeters(-Constants.TUNE_MODULES_DRIVE_SPEED),
    //                           Rotation2d.fromDegrees(90)),
    //                       new SwerveModuleState(
    //                           Units.feetToMeters(-Constants.TUNE_MODULES_DRIVE_SPEED),
    //                           Rotation2d.fromDegrees(90))
    //                     });
    //               },
    //               swerveDrive))
    //       .onFalse(
    //           new InstantCommand(
    //               () -> {
    //                 motionMode = MotionMode.FULL_DRIVE;
    //               },
    //               swerveDrive));
    // }

    // // cube: LeftBumper(5), tipped cone: RightTrigger(Axis 3), upright cone: RightBumper(6)
    // driver
    //     .leftBumper()
    //     .onTrue(
    //         new SequentialCommandGroup(
    //             new InstantCommand(
    //                 () -> {
    //                   gamePieceMode = GamePieceMode.CUBE;
    //                 }),
    //             Elevator.Commands.setToHeightAndWait(SuperstructureConstants.INTAKE_CUBE),
    //             new ParallelCommandGroup(
    //                 Intake.Commands.setTopVelocityRPM(
    //                     SuperstructureConstants.INTAKE_CUBE.getTopRPM()),
    //                 Intake.Commands.setBottomVelocityRPM(
    //                     SuperstructureConstants.INTAKE_CUBE.getBottomRPM()),
    //                 FourBar.Commands.setAngleDegAndWait(
    //                     SuperstructureConstants.INTAKE_CUBE.getFourBarPosition()),
    //                 LightStrip.Commands.setColorPattern(Pattern.Yellow)),
    //             new WaitUntilCommand(() -> intake.hasGamepiece()),
    //             FourBar.Commands.retract(),
    //             new ParallelCommandGroup(
    //                     new InstantCommand(() -> RumbleManager.getInstance().setDriver(1, 0.02))
    //                         .repeatedly(),
    //                     LightStrip.Commands.blinkAnyPattern(Pattern.DarkGreen))
    //                 .until(() -> fourBar.isAtTarget()),
    //             new ConditionalCommand(
    //                 LightStrip.Commands.setColorPattern(Pattern.DarkGreen),
    //                 new SequentialCommandGroup(
    //                     LightStrip.Commands.blinkAnyPattern(Pattern.Red),
    //                     new WaitCommand(2.0),
    //                     LightStrip.Commands.defaultColorPattern()),
    //                 () -> intake.hasGamepiece())))
    //     .onFalse(
    //         new SequentialCommandGroup(
    //             Elevator.Commands.elevatorCurrentHeight(),
    //             new ConditionalCommand(
    //                 new ParallelCommandGroup(
    //                     Intake.Commands.setTopVelocityRPM(
    //                         SuperstructureConstants.HOLD_CUBE.getTopRPM()),
    //                     Intake.Commands.setBottomVelocityRPM(
    //                         SuperstructureConstants.HOLD_CUBE.getBottomRPM())),
    //                 new ParallelCommandGroup(
    //                     Intake.Commands.setTopVelocityRPM(0),
    //                     Intake.Commands.setBottomVelocityRPM(0)),
    //                 () -> intake.hasGamepiece()),
    //             FourBar.Commands.setAngleDegAndWait(
    //                 Constants.FourBarConstants.IDLE_ANGLE_DEGREES)));

    // //      driver
    // //              .leftTrigger(0.25)
    // //              .onTrue(
    // //                      new SequentialCommandGroup(
    // //                              new InstantCommand(
    // //                                      () -> {
    // //                                          gamePieceMode = GamePieceMode.CUBE;
    // //                                      }),
    // //
    // // Elevator.Commands.setToHeightAndWait(SuperstructureConstants.INTAKE_CUBE_DEFLATED),
    // //                              new ParallelCommandGroup(
    // //                                      Intake.Commands.setTopVelocityRPM(
    // //
    // // SuperstructureConstants.INTAKE_CUBE_DEFLATED.getTopRPM()),
    // //                                      Intake.Commands.setBottomVelocityRPM(
    // //
    // // SuperstructureConstants.INTAKE_CUBE_DEFLATED.getBottomRPM()),
    // //                                      FourBar.Commands.setAngleDegAndWait(
    // //                                              SuperstructureConstants.INTAKE_CUBE_DEFLATED
    // // .getFourBarPosition()),
    // //                                      LightStrip.Commands.setColorPattern(Pattern.Yellow)),
    // //                              new WaitUntilCommand(() -> intake.hasGamepiece()),
    // //                              FourBar.Commands.retract(),
    // //                              new ParallelCommandGroup(
    // //                                      new InstantCommand(() ->
    // // RumbleManager.getInstance().setDriver(1, 0.02)).repeatedly(),
    // //
    // // LightStrip.Commands.blinkAnyPattern(Pattern.DarkGreen)).until(() -> fourBar.isAtTarget()),
    // //                              new ConditionalCommand(
    // //
    // LightStrip.Commands.setColorPattern(Pattern.DarkGreen),
    // //                                      new SequentialCommandGroup(
    // //
    // // LightStrip.Commands.blinkAnyPattern(Pattern.Red),
    // //                                              new WaitCommand(2.0),
    // //                                              LightStrip.Commands.defaultColorPattern()),
    // //                                      () ->intake.hasGamepiece())))
    // //              .onFalse(
    // //                      new SequentialCommandGroup(
    // //                              Elevator.Commands.elevatorCurrentHeight(),
    // //                              new ConditionalCommand(
    // //                                      new ParallelCommandGroup(
    // //                                              Intake.Commands.setTopVelocityRPM(
    // //
    // // SuperstructureConstants.HOLD_CUBE.getTopRPM()),
    // //                                              Intake.Commands.setBottomVelocityRPM(
    // //
    // // SuperstructureConstants.HOLD_CUBE.getBottomRPM())),
    // //                                      new ParallelCommandGroup(
    // //                                              Intake.Commands.setTopVelocityRPM(0),
    // //                                              Intake.Commands.setBottomVelocityRPM(0)),
    // //                                      () -> intake.hasGamepiece()),
    // //                              FourBar.Commands.retract()));

    // driver
    //     .rightTrigger(0.25)
    //     .onTrue(
    //         new SequentialCommandGroup(
    //             new InstantCommand(
    //                 () -> {
    //                   gamePieceMode = GamePieceMode.CONE;
    //                 }),
    //             Elevator.Commands.setToHeightAndWait(SuperstructureConstants.INTAKE_TIPPED_CONE),
    //             new ParallelCommandGroup(
    //                 Intake.Commands.setTopVelocityRPM(
    //                     SuperstructureConstants.INTAKE_TIPPED_CONE.getTopRPM()),
    //                 Intake.Commands.setBottomVelocityRPM(
    //                     SuperstructureConstants.INTAKE_TIPPED_CONE.getBottomRPM()),
    //                 FourBar.Commands.setAngleDegAndWait(
    //                     SuperstructureConstants.INTAKE_TIPPED_CONE.getFourBarPosition()),
    //                 LightStrip.Commands.setColorPattern(Pattern.Yellow)),
    //             new WaitUntilCommand(() -> intake.hasGamepiece()),
    //             FourBar.Commands.retract(),
    //             new ParallelCommandGroup(
    //                     new InstantCommand(() -> RumbleManager.getInstance().setDriver(1, 0.02))
    //                         .repeatedly(),
    //                     LightStrip.Commands.blinkAnyPattern(Pattern.DarkGreen))
    //                 .until(() -> fourBar.isAtTarget()),
    //             new ConditionalCommand(
    //                 LightStrip.Commands.setColorPattern(Pattern.DarkGreen),
    //                 new SequentialCommandGroup(
    //                     LightStrip.Commands.blinkAnyPattern(Pattern.Red),
    //                     new WaitCommand(2.0),
    //                     LightStrip.Commands.defaultColorPattern()),
    //                 () -> intake.hasGamepiece())))
    //     .onFalse(
    //         new SequentialCommandGroup(
    //             Elevator.Commands.elevatorCurrentHeight(),
    //             new ConditionalCommand(
    //                 new ParallelCommandGroup(
    //                     Intake.Commands.setTopVelocityRPM(
    //                         SuperstructureConstants.HOLD_CONE.getTopRPM()),
    //                     Intake.Commands.setBottomVelocityRPM(
    //                         SuperstructureConstants.HOLD_CONE.getBottomRPM())),
    //                 new ParallelCommandGroup(
    //                     Intake.Commands.setTopVelocityRPM(0),
    //                     Intake.Commands.setBottomVelocityRPM(0)),
    //                 () -> intake.hasGamepiece()),
    //             FourBar.Commands.setAngleDegAndWait(
    //                 Constants.FourBarConstants.IDLE_ANGLE_DEGREES)));

    // driver
    //     .rightBumper()
    //     .onTrue(
    //         new SequentialCommandGroup(
    //             new InstantCommand(
    //                 () -> {
    //                   gamePieceMode = GamePieceMode.CONE;
    //                 }),
    //
    // Elevator.Commands.setToHeightAndWait(SuperstructureConstants.INTAKE_UPRIGHT_CONE),
    //             new ParallelCommandGroup(
    //                 Intake.Commands.setTopVelocityRPM(
    //                     SuperstructureConstants.INTAKE_UPRIGHT_CONE.getTopRPM()),
    //                 Intake.Commands.setBottomVelocityRPM(
    //                     SuperstructureConstants.INTAKE_UPRIGHT_CONE.getBottomRPM()),
    //                 FourBar.Commands.setAngleDegAndWait(
    //                     SuperstructureConstants.INTAKE_UPRIGHT_CONE.getFourBarPosition()),
    //                 LightStrip.Commands.setColorPattern(Pattern.Yellow)),
    //             new WaitUntilCommand(() -> intake.hasGamepiece()),
    //             FourBar.Commands.retract(),
    //             new ParallelCommandGroup(
    //                     new InstantCommand(() -> RumbleManager.getInstance().setDriver(1, 0.02))
    //                         .repeatedly(),
    //                     LightStrip.Commands.blinkAnyPattern(Pattern.DarkGreen))
    //                 .until(() -> fourBar.isAtTarget()),
    //             new ConditionalCommand(
    //                 LightStrip.Commands.setColorPattern(Pattern.DarkGreen),
    //                 new SequentialCommandGroup(
    //                     LightStrip.Commands.blinkAnyPattern(Pattern.Red),
    //                     new WaitCommand(2.0),
    //                     LightStrip.Commands.defaultColorPattern()),
    //                 () -> intake.hasGamepiece())))
    //     .onFalse(
    //         new SequentialCommandGroup(
    //             Elevator.Commands.elevatorCurrentHeight(),
    //             new ConditionalCommand(
    //                 new ParallelCommandGroup(
    //                     Intake.Commands.setTopVelocityRPM(
    //                         SuperstructureConstants.HOLD_CONE.getTopRPM()),
    //                     Intake.Commands.setBottomVelocityRPM(
    //                         SuperstructureConstants.HOLD_CONE.getBottomRPM())),
    //                 new ParallelCommandGroup(
    //                     Intake.Commands.setTopVelocityRPM(0),
    //                     Intake.Commands.setBottomVelocityRPM(0)),
    //                 () -> intake.hasGamepiece()),
    //             FourBar.Commands.setAngleDegAndWait(
    //                 Constants.FourBarConstants.IDLE_ANGLE_DEGREES)));

    // driver
    //     .b()
    //     .onTrue(FourBar.Commands.setAngleDegAndWait(-15))
    //     .onFalse(
    //         new SequentialCommandGroup(
    //             new WaitCommand(0.0),
    //             FourBar.Commands.setDrawVolts(3).until(() -> fourBar.getLimitSwitch()),
    //             FourBar.Commands.setAngleDegAndWait(
    //                 Constants.FourBarConstants.IDLE_ANGLE_DEGREES)));

    // driver
    //     .a()
    //     .onTrue(
    //         new ConditionalCommand(
    //             // Past mid point
    //             new InstantCommand(
    //                 () -> {
    //                   motionMode = MotionMode.TRAJECTORY;
    //                   goHumanPlayer.regenerateTrajectory();
    //
    // TrajectoryController.getInstance().changePath(goHumanPlayer.getTrajectory());
    //                 }),
    //             // Mid point interior
    //             new InstantCommand(
    //                 () -> {
    //                   motionMode = MotionMode.TRAJECTORY;
    //                   goClosestGrid.changingPath();
    //                   goClosestGrid.regenerateTrajectory();
    //
    // TrajectoryController.getInstance().changePath(goClosestGrid.getTrajectory());
    //                 }),
    //             () -> RedHawkUtil.pastMidPoint(swerveDrive.getUsablePose())))
    //     .whileTrue(
    //         new ConditionalCommand(
    //             // Past mid point
    //             new RepeatCommand(
    //                 new InstantCommand(
    //                     () -> {
    //                       if (goHumanPlayer.hasElapsed()) {
    //                         TrajectoryController.getInstance()
    //                             .changePath(goHumanPlayer.getTrajectory());
    //                       }
    //                     })),

    //             // Mid point interior
    //             new RepeatCommand(
    //                 new InstantCommand(
    //                     () -> {
    //                       if (goClosestGrid.hasElapsed()) {
    //                         TrajectoryController.getInstance()
    //                             .changePath(goClosestGrid.getTrajectory());
    //                       }
    //                     })),
    //             () -> RedHawkUtil.pastMidPoint(swerveDrive.getUsablePose())))
    //     .onFalse(new InstantCommand(() -> motionMode = MotionMode.FULL_DRIVE));

    // driver.x().onTrue(new InstantCommand(() -> motionMode = MotionMode.LOCKDOWN));

    // driver
    //     .y()
    //     .whileTrue(
    //         Commands.sequence(
    //             new InstantCommand(
    //                 () -> {
    //                   intake.setScoring(true);
    //                 }),
    //             Intake.Commands.score()))
    //     .onFalse(
    //         new SequentialCommandGroup(
    //             new InstantCommand(
    //                 () -> {
    //                   intake.setScoring(false);
    //                 }),
    //             Intake.Commands.setTopVelocityRPM(Constants.zero),
    //             Intake.Commands.setBottomVelocityRPM(Constants.zero),
    //             new ConditionalCommand(
    //                 LightStrip.Commands.setColorPattern(Pattern.DarkGreen),
    //                 LightStrip.Commands.defaultColorPattern(),
    //                 () -> intake.hasGamepiece()),
    //             new WaitCommand(0.5)
    //             // Elevator.Commands.setTargetHeightAndWait(0),
    //             // LightStrip.Commands.setColorPattern(DarkGreen)
    //             ));

    // driver
    //     .start()
    //     .onTrue(
    //         new InstantCommand(
    //             () -> {
    //               swerveDrive.resetGyro(Rotation2d.fromDegrees(0));
    //             }));

    // driver
    //     .back()
    //     .onTrue(
    //         new InstantCommand(
    //             () -> {
    //               swerveDrive.resetGyro(Rotation2d.fromDegrees(180));
    //             }));

    // // Operator Buttons
    // // high: Y(4), mid B(2):, low: A(1), home 4bar: START(8)

    // operator
    //     .y()
    //     .onTrue(
    //         new SequentialCommandGroup(
    //             Elevator.Commands.conditionalElevatorHigh(),
    //             FourBar.Commands.conditionalFourbarHigh(),
    //             new ConditionalCommand(
    //                 LightStrip.Commands.setColorPattern(Pattern.DarkGreen),
    //                 LightStrip.Commands.defaultColorPattern(),
    //                 () -> intake.hasGamepiece())));

    // operator
    //     .b()
    //     .onTrue(
    //         new SequentialCommandGroup(
    //             Elevator.Commands.conditionalElevatorMid(),
    //             FourBar.Commands.conditionalFourbarMid(),
    //             new ConditionalCommand(
    //                 LightStrip.Commands.setColorPattern(Pattern.DarkGreen),
    //                 LightStrip.Commands.defaultColorPattern(),
    //                 () -> intake.hasGamepiece())));
    // operator
    //     .a()
    //     .onTrue(
    //         new SequentialCommandGroup(
    //             Elevator.Commands.conditionalElevatorLow(),
    //             FourBar.Commands.conditionalFourbarLow(),
    //             new ConditionalCommand(
    //                 LightStrip.Commands.setColorPattern(Pattern.DarkGreen),
    //                 LightStrip.Commands.defaultColorPattern(),
    //                 () -> intake.hasGamepiece())));
    // operator
    //     .rightBumper()
    //     .and(operator.y())
    //     .onTrue(
    //         new SequentialCommandGroup(
    //             Elevator.Commands.setToHeightAndWait(SuperstructureConstants.SCORE_CONE_HIGH),
    //             FourBar.Commands.setAngleDegAndWait(SuperstructureConstants.SCORE_CONE_HIGH),
    //             new ConditionalCommand(
    //                 LightStrip.Commands.setColorPattern(Pattern.DarkGreen),
    //                 LightStrip.Commands.defaultColorPattern(),
    //                 () -> intake.hasGamepiece())));
    // operator
    //     .rightBumper()
    //     .and(operator.b())
    //     .onTrue(
    //         new SequentialCommandGroup(
    //             Elevator.Commands.setToHeightAndWait(SuperstructureConstants.SCORE_CONE_MID),
    //             FourBar.Commands.setAngleDegAndWait(SuperstructureConstants.SCORE_CONE_MID),
    //             new ConditionalCommand(
    //                 LightStrip.Commands.setColorPattern(Pattern.DarkGreen),
    //                 LightStrip.Commands.defaultColorPattern(),
    //                 () -> intake.hasGamepiece())));
    // operator
    //     .rightBumper()
    //     .and(operator.a())
    //     .onTrue(
    //         new ParallelCommandGroup(
    //             Elevator.Commands.setToHeightAndWait(SuperstructureConstants.SCORE_CONE_LOW),
    //             FourBar.Commands.setAngleDegAndWait(SuperstructureConstants.SCORE_CONE_LOW),
    //             new ConditionalCommand(
    //                 LightStrip.Commands.setColorPattern(Pattern.DarkGreen),
    //                 LightStrip.Commands.defaultColorPattern(),
    //                 () -> intake.hasGamepiece())));
    // operator
    //     .leftBumper()
    //     .and(operator.y())
    //     .onTrue(
    //         new ParallelCommandGroup(
    //             Elevator.Commands.setToHeightAndWait(SuperstructureConstants.SCORE_CUBE_HIGH),
    //             FourBar.Commands.setAngleDegAndWait(SuperstructureConstants.SCORE_CUBE_HIGH),
    //             new ConditionalCommand(
    //                 LightStrip.Commands.setColorPattern(Pattern.DarkGreen),
    //                 LightStrip.Commands.defaultColorPattern(),
    //                 () -> intake.hasGamepiece())));
    // operator
    //     .leftBumper()
    //     .and(operator.b())
    //     .onTrue(
    //         new ParallelCommandGroup(
    //             Elevator.Commands.setToHeightAndWait(SuperstructureConstants.SCORE_CUBE_MID),
    //             FourBar.Commands.setAngleDegAndWait(SuperstructureConstants.SCORE_CUBE_MID),
    //             new ConditionalCommand(
    //                 LightStrip.Commands.setColorPattern(Pattern.DarkGreen),
    //                 LightStrip.Commands.defaultColorPattern(),
    //                 () -> intake.hasGamepiece())));
    // operator
    //     .leftBumper()
    //     .and(operator.a())
    //     .onTrue(
    //         new ParallelCommandGroup(
    //             Elevator.Commands.setToHeightAndWait(SuperstructureConstants.SCORE_CUBE_LOW),
    //             FourBar.Commands.setAngleDegAndWait(SuperstructureConstants.SCORE_CUBE_LOW),
    //             new ConditionalCommand(
    //                 LightStrip.Commands.setColorPattern(Pattern.DarkGreen),
    //                 LightStrip.Commands.defaultColorPattern(),
    //                 () -> intake.hasGamepiece())));
    // operator
    //     .rightBumper()
    //     .and(operator.x())
    //     .onTrue(
    //         new SequentialCommandGroup(
    //             new InstantCommand(
    //                 () -> {
    //                   gamePieceMode = GamePieceMode.CONE;
    //                 }),
    //             Elevator.Commands.setToHeightAndWait(SuperstructureConstants.INTAKE_SHELF_CONE),
    //             new ParallelCommandGroup(
    //                 Intake.Commands.setTopVelocityRPM(
    //                     SuperstructureConstants.INTAKE_SHELF_CONE.getTopRPM()),
    //                 Intake.Commands.setBottomVelocityRPM(
    //                     SuperstructureConstants.INTAKE_SHELF_CONE.getBottomRPM()),
    //                 FourBar.Commands.setAngleDegAndWait(
    //                     SuperstructureConstants.INTAKE_SHELF_CONE.getFourBarPosition()))));

    // operator
    //     .povUp()
    //     .or(operator.povUpLeft())
    //     .or(operator.povUpRight())
    //     .onTrue(
    //         new SequentialCommandGroup(
    //             new InstantCommand(
    //                 () -> {
    //                   gamePieceMode = GamePieceMode.CONE;
    //                 }),
    //             Elevator.Commands.setToHeightAndWait(SuperstructureConstants.INTAKE_SHELF_CONE),
    //             new ParallelCommandGroup(
    //                 Intake.Commands.setTopVelocityRPM(
    //                     SuperstructureConstants.INTAKE_SHELF_CONE.getTopRPM()),
    //                 Intake.Commands.setBottomVelocityRPM(
    //                     SuperstructureConstants.INTAKE_SHELF_CONE.getBottomRPM()),
    //                 FourBar.Commands.setAngleDegAndWait(
    //                     SuperstructureConstants.INTAKE_SHELF_CONE.getFourBarPosition())),
    //             new WaitUntilCommand(() -> intake.hasGamepiece()),
    //             FourBar.Commands.retract()));

    // operator.start().onTrue(FourBar.Commands.reset());

    // operator
    //     .rightStick()
    //     .onTrue(
    //         new InstantCommand(
    //             () -> {
    //               //   swerveDrive.seed();
    //             }));

    // operator
    //     .axisGreaterThan(XboxController.Axis.kLeftX.value, 0.1)
    //     .onTrue(
    //         new InstantCommand(
    //             () -> {
    //               fourBar.setMode(FourBarMode.OPEN_LOOP);
    //             }));

    // operator
    //     .axisGreaterThan(XboxController.Axis.kLeftY.value, 0.1)
    //     .onTrue(
    //         new InstantCommand(
    //             () -> {
    //               elevator.manualControl = true;
    //             }));

    // operator
    //     .back()
    //     .onTrue(
    //         new InstantCommand(
    //             () -> {
    //               fourBar.reseed();
    //             }));

    // operator
    //     .povDown()
    //     .or(operator.povDownLeft())
    //     .or(operator.povDownRight())
    //     .onTrue(
    //         new ParallelCommandGroup(
    //             Elevator.Commands.setToHeightAndWait(0),
    //             FourBar.Commands.retract(),
    //             LightStrip.Commands.defaultColorPattern(),
    //             new ConditionalCommand(
    //                 LightStrip.Commands.setColorPattern(Pattern.DarkGreen),
    //                 LightStrip.Commands.defaultColorPattern(),
    //                 () -> intake.hasGamepiece()),
    //             new ConditionalCommand(
    //                 Commands.parallel(
    //                     Intake.Commands.setTopVelocityRPM(
    //                         SuperstructureConstants.HOLD_CONE.getTopRPM()),
    //                     Intake.Commands.setBottomVelocityRPM(
    //                         SuperstructureConstants.HOLD_CONE.getBottomRPM())),
    //                 new ParallelCommandGroup(
    //                     Intake.Commands.setTopVelocityRPM(0),
    //                     Intake.Commands.setBottomVelocityRPM(0)),
    //                 () -> intake.hasGamepiece())));

    // operator.rightTrigger(0.25).onTrue(LightStrip.Commands.setColorPattern(Pattern.StrobeGold));
    // operator.leftTrigger(0.25).onTrue(LightStrip.Commands.setColorPattern(Pattern.StrobeBlue));

    // //
    // operator.start().onTrue(Slapper.Commands.sendIt()).onFalse(Slapper.Commands.comeBackHome());
    // // operator
    // //     .axisLessThan(1, -0.1)
    // //     .whileTrue(
    // //         new RepeatCommand(
    // //             new InstantCommand(
    // //                 () -> {
    // //                   double targetHeightInches =
    // //                       elevator.getCurrentHeight() + (10 * ((-1) *
    // operator.getRawAxis(1)));
    // //                   if (!(targetHeightInches
    // //                       > (Constants.ElevatorConstants.ELEVATOR_MAX_HEIGHT_INCHES))) {
    // //                     elevator.setTargetHeight(targetHeightInches);
    // //                   }
    // //                 })));

    // // operator
    // //     .axisGreaterThan(1, 0.1)
    // //     .whileTrue(
    // //         new RepeatCommand(
    // //             new InstantCommand(
    // //                 () -> {
    // //                   double targetHeightInches =
    // //                       elevator.getCurrentHeight() - (10 * operator.getRawAxis(1));
    // //                   if (!(targetHeightInches
    // //                       > (Constants.ElevatorConstants.ELEVATOR_MAX_HEIGHT_INCHES))) {
    // //                     elevator.setTargetHeight(targetHeightInches);
    // //                   }
    // //                 })));

    // // operator
    // //     .axisLessThan(5, -0.1)
    // //     .whileTrue(
    // //         new RepeatCommand(
    // //             (new InstantCommand(
    // //                 () -> {
    // //                   double targetDegs = fourBar.getCurrentDegs() + (20 *
    // // operator.getRawAxis(5));
    // //                   if (!(targetDegs < Constants.FourBarConstants.EXTENDED_ANGLE_DEGREES
    // //                       || targetDegs > Constants.FourBarConstants.RETRACTED_ANGLE_DEGREES))
    // {
    // //                     fourBar.setAngleDeg(targetDegs);
    // //                   }
    // //                 }))));

    // // operator
    // //     .axisGreaterThan(5, 0.1)
    // //     .whileTrue(
    // //             (new InstantCommand(
    // //                 () -> {
    // //                   double targetDegs = fourBar.getCurrentDegs() + (20 *
    // // operator.getRawAxis(5));
    // //                   if (!(targetDegs < Constants.FourBarConstants.EXTENDED_ANGLE_DEGREES
    // //                       || targetDegs > Constants.FourBarConstants.RETRACTED_ANGLE_DEGREES))
    // {
    // //                     fourBar.setAngleDeg(targetDegs);
    // //                   }
    // //                 })));

    // if (!Robot.isReal()) {
    //   DriverStation.silenceJoystickConnectionWarning(true);
    // }
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    // ErrHandler.getInstance().log();
    // RumbleManager.getInstance().periodic();
    // // mechManager.periodic();
    // if (Math.abs(driver.getRightX()) > 0.25) {
    //   motionMode = MotionMode.FULL_DRIVE;
    // }

    // // swerveDrive.seed();

    // RoboRioSim.setVInVoltage(
    //     BatterySim.calculateDefaultBatteryLoadedVoltage(
    //         elevator.getCurrentDraw()
    //             + fourBar.getCurrentDraw()
    //             + intake.getCurrentDraw()
    //             + swerveDrive.getTotalCurrentDraw()));

    // Logger.getInstance().recordOutput("Game piece mode", gamePieceMode.name());
    // Logger.getInstance()
    //     .recordOutput(
    //         "Filtered CAN Utilization",
    //
    // canUtilizationFilter.calculate(RobotController.getCANStatus().percentBusUtilization));
    // Logger.getInstance()
    //     .recordOutput(
    //         "Memory Usage",
    //         (Runtime.getRuntime().totalMemory() - Runtime.getRuntime().freeMemory())
    //             / 1024.0
    //             / 1024.0);

    // TimestampedDoubleArray[] frontfQueue = frontVisionPose.readQueue();
    // TimestampedDoubleArray[] frontcQueue = frontCamera2TagPose.readQueue();

    // TimestampedDoubleArray[] rearfQueue = rearVisionPose.readQueue();
    // TimestampedDoubleArray[] rearcQueue = rearCamera2TagPose.readQueue();

    // if (driver.getRightX() > 0.5) {
    //   motionMode = MotionMode.FULL_DRIVE;
    // }

    // if (frontfQueue.length > 0
    //     && frontcQueue.length > 0
    //     && vision.hasMultipleTargets(Limelights.FRONT)) {
    //   TimestampedDoubleArray fLastCameraReading = frontfQueue[frontfQueue.length - 1];
    //   TimestampedDoubleArray cLastCameraReading = frontcQueue[frontcQueue.length - 1];
    //   swerveDrive.updateVisionPose(fLastCameraReading, cLastCameraReading);
    // } else if (rearfQueue.length > 0
    //     && rearcQueue.length > 0
    //     && vision.hasMultipleTargets(Limelights.REAR)) {
    //   TimestampedDoubleArray fLastCameraReading = rearfQueue[rearfQueue.length - 1];
    //   TimestampedDoubleArray cLastCameraReading = rearcQueue[rearcQueue.length - 1];
    //   swerveDrive.updateVisionPose(fLastCameraReading, cLastCameraReading);
    // }
  }

  @Override
  public void disabledInit() {
    // if (autoCommand != null) {
    //   autoCommand.cancel();
    // }
    // swerveDrive.seed();

    // Robot.motionMode = MotionMode.LOCKDOWN;

    // vision.setCurrentSnapshotMode(SnapshotMode.OFF);
  }

  @Override
  public void disabledPeriodic() {
    // checkAlliance();

    // swerveDrive.seed();
    // elevator.resetController();
    // fourBar.reset();
    // SmartDashboard.putBoolean("Driver Controller OK", DriverStation.getJoystickIsXbox(0));
    // SmartDashboard.putBoolean("Operator Controller OK", DriverStation.getJoystickIsXbox(1));
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    checkAlliance();
    motionMode = MotionMode.TRAJECTORY;
    autoCommand = autoChooser.get();

    if (autoCommand != null) {
      autoCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (autoCommand != null) {
      autoCommand.cancel();
    }
    Robot.motionMode = MotionMode.FULL_DRIVE;
    // Autos.clearAll();
    // AutoPath.Autos.clearAll();

    vision.setCurrentSnapshotMode(SnapshotMode.TWO_PER_SECOND);
  }

  // grab botpose from the network table, put it into swerve drive inputs, read
  // botpose, and put
  // that into the pose estimator
  // using the vision command

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    swerveDrive.zeroGyro();
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  public void buildAutoChooser() {
    SwerveSubsystem.allianceFlipper = DriverStation.getAlliance() == Alliance.Red ? -1 : 1;
    autoChooser.addDefaultOption("ConeCubeConeOver", new ConeCubeConeOver());
    autoChooser.addOption("ThreeCubeOver", new ThreeCubeOver());
    // autoChooser.addOption("SlapConeCubeConeOver", new SlapConeCubeConeOver());
    autoChooser.addOption("FastThreeCubeOver", new FastThreeCubeOver());
    autoChooser.addOption("OneConeTwoCubeOver", new OneConeTwoCubeOver());
    autoChooser.addOption("TwoConeOver", new TwoConeOver());
    autoChooser.addOption("TwoCubeOver", new TwoCubeOver());
    autoChooser.addOption("Bridge", new GetOnBridge(true));
    autoChooser.addOption("PID Bridge", new PIDOnBridge(true));
    autoChooser.addOption("OneCubeOverBridge", new OneCubeOverBridge());
    autoChooser.addOption("MobilityBridge", new MobilityBridge());
    autoChooser.addOption("MobilityBridgePathing", new MobilityBridgePathing());
    autoChooser.addOption("TwoCubeOverBridge", new TwoCubeOverBridge());
    autoChooser.addOption("OneConeBridge", new OneConeBridge());
    autoChooser.addOption("ChargeTestCommand", new OnBridgeUntilMovement(true));
    autoChooser.addOption("TwoConeUnder", new TwoConeUnder());
    autoChooser.addOption("CommunityScoring", new ScoreCommunityUnder());
    autoChooser.addOption("OneConeOneCubeUnder", new OneConeOneCubeUnder());
    autoChooser.addOption("OneConeOneCubeUnderHigh", new OneConeOneCubeUnderHigh());
    autoChooser.addOption("SimpleConeMiddle", new SimpleCone());
    autoChooser.addOption(
        "FiveToBridge",
        new SequentialCommandGroup(
            new InstantCommand(
                () ->
                    swerveDrive.resetOdometry(
                        AutoPath.Autos.FIVE_ON_BRIDGE.getTrajectory().getInitialHolonomicPose())),
            SwerveSubsystem.Commands.stringTrajectoriesTogether(
                AutoPath.Autos.FIVE_ON_BRIDGE.getTrajectory())));
  }

  public void checkAlliance() {
    Alliance checkedAlliance = DriverStation.getAlliance();
    Logger.getInstance().recordOutput("DS Alliance", currentAlliance.name());

    if (DriverStation.isDSAttached() && checkedAlliance != currentAlliance) {
      currentAlliance = checkedAlliance;

      // these gyro resets are mostly for ironing out teleop driving issues

      // if we are on blue, we are probably facing towards the blue DS, which is -x.
      // that corresponds to a 180 deg heading.
      if (checkedAlliance == Alliance.Blue) {
        swerveDrive.resetGyro(Rotation2d.fromDegrees(180));
      }

      // if we are on red, we are probably facing towards the red DS, which is +x.
      // that corresponds to a 0 deg heading.
      if (checkedAlliance == Alliance.Red) {
        swerveDrive.resetGyro(Rotation2d.fromDegrees(0));
      }

      goClosestGrid = new GoClosestGrid();
      buildAutoChooser();
    }
  }

  public String goFast() {
    return "nyoooooooooom";
  }

  public String goSlow() {
    return "...nyom...";
  }
}
