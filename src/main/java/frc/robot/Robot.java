// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.LightStrip.Pattern.*;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.commands.OTF.GoClosestGrid;
import frc.robot.commands.fullRoutines.TwoCargoOver;
import frc.robot.commands.fullRoutines.TwoCargoUnder;
import frc.robot.subsystems.LightStrip;
import frc.robot.subsystems.elevatorIO.Elevator;
import frc.robot.subsystems.elevatorIO.ElevatorIOSim;
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
import frc.robot.subsystems.visionIO.VisionIOSim;
import frc.robot.subsystems.visionIO.VisionLimelight;
import frc.robot.util.DebugMode;
import frc.robot.util.MechanismManager;
import frc.robot.util.MotionHandler.MotionMode;
import frc.robot.util.RedHawkUtil.ErrHandler;
import frc.robot.util.SwerveHeadingController;
import frc.robot.util.TrajectoryController;
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
  public static SwerveSubsystem swerveDrive;
  public GoClosestGrid goClosestGrid;
  public static LightStrip lights;
  private Command autoCommand;
  public static GamePieceMode gamePieceMode = GamePieceMode.CUBE;
  private LinearFilter canUtilizationFilter = LinearFilter.singlePoleIIR(0.1, 0.02);

  public static final CommandXboxController driver =
      new CommandXboxController(Constants.RobotMap.DRIVER_PORT);
  public static final CommandXboxController operator =
      new CommandXboxController(Constants.RobotMap.OPERATOR_PORT);

  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Autonomous Routine");

  public static double[] poseValue;
  DoubleArraySubscriber visionPose;

  @Override
  public void robotInit() {
    CameraServer.startAutomaticCapture();
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    visionPose = table.getDoubleArrayTopic("botpose").subscribe(new double[] {});
    Logger.getInstance().addDataReceiver(new NT4Publisher());
    Logger.getInstance().recordMetadata("GitRevision", Integer.toString(GVersion.GIT_REVISION));
    Logger.getInstance().recordMetadata("GitSHA", GVersion.GIT_SHA);
    Logger.getInstance().recordMetadata("GitDate", GVersion.GIT_DATE);
    Logger.getInstance().recordMetadata("GitBranch", GVersion.GIT_BRANCH);
    Logger.getInstance().recordMetadata("BuildDate", GVersion.BUILD_DATE);
    if (isReal()) {
      Logger.getInstance().addDataReceiver(new WPILOGWriter("/media/sda1"));
    }

    Logger.getInstance().start();

    fourBar = new FourBar(isSimulation() ? new FourBarIOSim() : new FourBarIOSparks());
    // elevator = new Elevator(isSimulation() ? new ElevatorIOSim() : new ElevatorIOSparks());
    elevator = new Elevator(new ElevatorIOSim());
    intake = new Intake(isSimulation() ? new IntakeIOSim() : new IntakeIOSparks());
    vision = new Vision(isSimulation() ? new VisionIOSim() : new VisionLimelight());
    lights = new LightStrip();

    swerveDrive =
        isSimulation()
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

    // fourBar.setDefaultCommand(
    // new InstantCommand(
    // () ->
    // fourBar.setAngleDeg(
    // MathUtil.clamp(
    // fourBar.getCurrentDegs()
    // + (MathUtil.applyDeadband(
    // -operator.getLeftY(),
    // Constants.DriveConstants.K_JOYSTICK_TURN_DEADZONE)
    // / 10),
    // Units.radiansToDegrees(FourBarConstants.MAX_ANGLE_RADIANS),
    // Units.radiansToDegrees(FourBarConstants.RETRACTED_ANGLE_RADIANS))),
    // fourBar));

    // elevator.setDefaultCommand(
    // new InstantCommand(
    // () ->
    // elevator.setTargetHeight(
    // MathUtil.clamp(
    // elevator.getTargetHeight()
    // + (MathUtil.applyDeadband(
    // -operator.getRightY(),
    // Constants.DriveConstants.K_JOYSTICK_TURN_DEADZONE)
    // / 10),
    // Constants.zero,
    // Units.metersToFeet(ElevatorConstants.ELEVATOR_MAX_HEIGHT_METERS))),
    // elevator));

    // lights.setDefaultCommand(LightStrip.Commands.defaultColorPattern());

    mechManager = new MechanismManager();
    autoCommand = new TwoCargoOver();
    goClosestGrid = new GoClosestGrid();

    autoChooser.addOption("TwoBridgeOver", new TwoCargoOver());
    autoChooser.addOption("TwoBridgeUnder", new TwoCargoUnder());

    // Driver Controls
    if (Constants.DEBUG_MODE == DebugMode.MATCH) {
      driver
          .povUp()
          .onTrue(
              new InstantCommand(
                  () -> {
                    motionMode = MotionMode.HEADING_CONTROLLER;
                    SwerveHeadingController.getInstance().setSetpoint(Rotation2d.fromDegrees(0));
                  }));

      driver
          .povLeft()
          .onTrue(
              new InstantCommand(
                  () -> {
                    motionMode = MotionMode.HEADING_CONTROLLER;
                    SwerveHeadingController.getInstance().setSetpoint(Rotation2d.fromDegrees(90));
                  }));

      driver
          .povDown()
          .onTrue(
              new InstantCommand(
                  () -> {
                    motionMode = MotionMode.HEADING_CONTROLLER;
                    SwerveHeadingController.getInstance().setSetpoint(Rotation2d.fromDegrees(180));
                  }));

      driver
          .povRight()
          .onTrue(
              new InstantCommand(
                  () -> {
                    motionMode = MotionMode.HEADING_CONTROLLER;
                    SwerveHeadingController.getInstance().setSetpoint(Rotation2d.fromDegrees(270));
                  }));
    } else if (Constants.DEBUG_MODE == DebugMode.TUNE_MODULES) {
      driver
          .povUp()
          .whileTrue(
              new InstantCommand(
                  () -> {
                    motionMode = MotionMode.NULL;
                    swerveDrive.setModuleStates(
                        new SwerveModuleState[] {
                          new SwerveModuleState(
                              Units.feetToMeters(Constants.TUNE_MODULES_DRIVE_SPEED),
                              Rotation2d.fromDegrees(0)),
                          new SwerveModuleState(
                              Units.feetToMeters(Constants.TUNE_MODULES_DRIVE_SPEED),
                              Rotation2d.fromDegrees(0)),
                          new SwerveModuleState(
                              Units.feetToMeters(Constants.TUNE_MODULES_DRIVE_SPEED),
                              Rotation2d.fromDegrees(0)),
                          new SwerveModuleState(
                              Units.feetToMeters(Constants.TUNE_MODULES_DRIVE_SPEED),
                              Rotation2d.fromDegrees(0))
                        });
                  },
                  swerveDrive))
          .onFalse(
              new InstantCommand(
                  () -> {
                    motionMode = MotionMode.FULL_DRIVE;
                  },
                  swerveDrive));
      driver
          .povDown()
          .whileTrue(
              new InstantCommand(
                  () -> {
                    motionMode = MotionMode.NULL;
                    swerveDrive.setModuleStates(
                        new SwerveModuleState[] {
                          new SwerveModuleState(
                              Units.feetToMeters(-Constants.TUNE_MODULES_DRIVE_SPEED),
                              Rotation2d.fromDegrees(90)),
                          new SwerveModuleState(
                              Units.feetToMeters(-Constants.TUNE_MODULES_DRIVE_SPEED),
                              Rotation2d.fromDegrees(90)),
                          new SwerveModuleState(
                              Units.feetToMeters(-Constants.TUNE_MODULES_DRIVE_SPEED),
                              Rotation2d.fromDegrees(90)),
                          new SwerveModuleState(
                              Units.feetToMeters(-Constants.TUNE_MODULES_DRIVE_SPEED),
                              Rotation2d.fromDegrees(90))
                        });
                  },
                  swerveDrive))
          .onFalse(
              new InstantCommand(
                  () -> {
                    motionMode = MotionMode.FULL_DRIVE;
                  },
                  swerveDrive));
    }

    driver
        .leftBumper()
        .onTrue(
            new SequentialCommandGroup(
                new InstantCommand(
                    () -> {
                      gamePieceMode = GamePieceMode.CUBE;
                    }),
                Elevator.Commands.setTargetHeightAndWait(
                    SuperstructureConstants.INTAKE_CUBE.getElevatorPosition()),
                new ParallelCommandGroup(
                    Intake.Commands.setWheelVelocityRPM(
                        SuperstructureConstants.INTAKE_CUBE.getWheelRPM()),
                    Intake.Commands.setRollerVelocityRPM(
                        SuperstructureConstants.INTAKE_CUBE.getRollerRPM()),
                    FourBar.Commands.setAngleDegAndWait(
                        SuperstructureConstants.INTAKE_CUBE.getFourBarPosition()))))
        .onFalse(
            new SequentialCommandGroup(
                Elevator.Commands.elevatorCurrentHeight(),
                new WaitCommand(0.5),
                Intake.Commands.setWheelVelocityRPM(Constants.zero),
                Intake.Commands.setRollerVelocityRPM(Constants.zero),
                FourBar.Commands.retract()));

    driver
        .rightTrigger(0.25)
        .onTrue(
            new SequentialCommandGroup(
                new InstantCommand(
                    () -> {
                      gamePieceMode = GamePieceMode.CONE;
                    }),
                Elevator.Commands.setTargetHeightAndWait(
                    SuperstructureConstants.INTAKE_TIPPED_CONE.getElevatorPosition()),
                new ParallelCommandGroup(
                    Intake.Commands.setWheelVelocityRPM(
                        SuperstructureConstants.INTAKE_TIPPED_CONE.getWheelRPM()),
                    Intake.Commands.setRollerVelocityRPM(
                        SuperstructureConstants.INTAKE_TIPPED_CONE.getRollerRPM()),
                    FourBar.Commands.setAngleDegAndWait(
                        SuperstructureConstants.INTAKE_TIPPED_CONE.getFourBarPosition()))))
        .onFalse(
            new SequentialCommandGroup(
                Elevator.Commands.elevatorCurrentHeight(),
                new WaitCommand(0.5),
                Intake.Commands.setWheelVelocityRPM(Constants.zero),
                Intake.Commands.setRollerVelocityRPM(Constants.zero),
                FourBar.Commands.retract()));

    driver
        .rightBumper()
        .onTrue(
            new SequentialCommandGroup(
                new InstantCommand(
                    () -> {
                      gamePieceMode = GamePieceMode.CONE;
                    }),
                Elevator.Commands.setTargetHeightAndWait(
                    SuperstructureConstants.INTAKE_UPRIGHT_CONE.getElevatorPosition()),
                new ParallelCommandGroup(
                    Intake.Commands.setWheelVelocityRPM(
                        SuperstructureConstants.INTAKE_UPRIGHT_CONE.getWheelRPM()),
                    Intake.Commands.setRollerVelocityRPM(
                        SuperstructureConstants.INTAKE_UPRIGHT_CONE.getRollerRPM()),
                    FourBar.Commands.setAngleDegAndWait(
                        SuperstructureConstants.INTAKE_UPRIGHT_CONE.getFourBarPosition()))))
        .onFalse(
            new SequentialCommandGroup(
                Elevator.Commands.elevatorCurrentHeight(),
                new WaitCommand(0.5),
                Intake.Commands.setWheelVelocityRPM(-500),
                Intake.Commands.setRollerVelocityRPM(-500),
                FourBar.Commands.retract()));

    driver
        .b()
        .onTrue(FourBar.Commands.setAngleDegAndWait(15))
        .onFalse(new SequentialCommandGroup(new WaitCommand(0.5), FourBar.Commands.retract()));

    driver
        .a()
        .onTrue(
            new InstantCommand(
                () -> {
                  motionMode = MotionMode.TRAJECTORY;
                  goClosestGrid.changingPath();
                  goClosestGrid.regenerateTrajectory();
                  TrajectoryController.getInstance().changePath(goClosestGrid.getTrajectory());
                }))
        .whileTrue(
            new RepeatCommand(
                new InstantCommand(
                    () -> {
                      if (goClosestGrid.hasElapsed()) {
                        TrajectoryController.getInstance()
                            .changePath(goClosestGrid.getTrajectory());
                      }
                    })))
        .onFalse(new InstantCommand(() -> motionMode = MotionMode.FULL_DRIVE));

    driver.x().onTrue(new InstantCommand(() -> motionMode = MotionMode.LOCKDOWN));

    driver
        .y()
        .whileTrue(
            new ParallelCommandGroup(
                Intake.Commands.setRollerVelocityRPM(
                    SuperstructureConstants.SCORE.getRollerRPM(), gamePieceMode),
                Intake.Commands.setWheelVelocityRPM(
                    SuperstructureConstants.SCORE.getWheelRPM(), gamePieceMode)))
        .onFalse(
            new SequentialCommandGroup(
                Intake.Commands.setRollerVelocityRPM(Constants.zero),
                Intake.Commands.setWheelVelocityRPM(Constants.zero),
                new WaitCommand(0.5),
                // Elevator.Commands.setTargetHeightAndWait(0),
                LightStrip.Commands.setColorPattern(DarkGreen)));

    // Operator Buttons
    operator
        .rightBumper()
        .and(operator.y())
        .onTrue(
            new SequentialCommandGroup(
                Elevator.Commands.elevatorConeHighScoreAndWait(), FourBar.Commands.extend()));

    operator
        .rightBumper()
        .and(operator.b())
        .onTrue(
            new SequentialCommandGroup(
                Elevator.Commands.elevatorConeMidScoreAndWait(), FourBar.Commands.extend()));

    operator
        .rightBumper()
        .and(operator.a())
        .onTrue(
            new SequentialCommandGroup(
                Elevator.Commands.elevatorConeLowScoreAndWait(), FourBar.Commands.extend()));

    operator
        .leftBumper()
        .and(operator.y())
        .onTrue(
            new SequentialCommandGroup(
                Elevator.Commands.elevatorCubeHighScoreAndWait(), FourBar.Commands.extend()));

    operator
        .leftBumper()
        .and(operator.b())
        .onTrue(
            new SequentialCommandGroup(
                Elevator.Commands.elevatorCubeMidScoreAndWait(), FourBar.Commands.extend()));

    operator
        .leftBumper()
        .and(operator.a())
        .onTrue(
            new SequentialCommandGroup(
                Elevator.Commands.elevatorCubeLowScoreAndWait(), FourBar.Commands.extend()));

    operator
        .povDown()
        .onTrue(
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    Elevator.Commands.setTargetHeightAndWait(0).withTimeout(2.0),
                    FourBar.Commands.retract()),
                new InstantCommand(
                    () -> {
                      elevator.resetencoders();
                    })));

    operator.rightTrigger(0.25).onTrue(LightStrip.Commands.setColorPattern(Yellow));
    operator.leftTrigger(0.25).onTrue(LightStrip.Commands.setColorPattern(Purple));

    if (!Robot.isReal()) {
      DriverStation.silenceJoystickConnectionWarning(true);
    }
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    ErrHandler.getInstance().log();
    mechManager.periodic();
    if (Math.abs(driver.getRightX()) > 0.25) {
      motionMode = MotionMode.FULL_DRIVE;
    }

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(
            elevator.getCurrentDraw()
                + fourBar.getCurrentDraw()
                + intake.getCurrentDraw()
                + swerveDrive.getTotalCurrentDraw()));

    Logger.getInstance().recordOutput("Game piece mode", gamePieceMode.name());
    Logger.getInstance()
        .recordOutput(
            "Filtered CAN Utilization",
            canUtilizationFilter.calculate(RobotController.getCANStatus().percentBusUtilization));
  }

  @Override
  public void disabledInit() {
    if (autoCommand != null) {
      autoCommand.cancel();
    }

    Robot.motionMode = MotionMode.LOCKDOWN;
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
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
  }

  // grab botpose from the network table, put it into swerve drive inputs, read
  // botpose, and put
  // that into the pose estimator
  // using the vision command

  @Override
  public void teleopPeriodic() {
    TimestampedDoubleArray[] queue = visionPose.readQueue();

    if (driver.getRightX() > 0.5) {
      motionMode = MotionMode.FULL_DRIVE;
    }

    if (queue.length > Constants.zero) {
      TimestampedDoubleArray lastCameraReading = queue[queue.length - 1];
      swerveDrive.updateVisionPose(lastCameraReading);
    }
  }

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

  public String goFast() {
    return "nyoooooooooom";
  }

  public String goSlow() {
    return "...nyom...";
  }
}
