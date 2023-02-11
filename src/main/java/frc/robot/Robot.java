// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.LightStrip.Pattern.*;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.fullRoutines.OneToAToThreeToBridge;
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
import frc.robot.subsystems.visionIO.VisionIOSim;
import frc.robot.subsystems.visionIO.VisionLimelight;
import frc.robot.util.MechanismManager;
import frc.robot.util.MotionHandler.MotionMode;
import frc.robot.util.RedHawkUtil.ErrHandler;
import frc.robot.util.SwerveHeadingController;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

public class Robot extends LoggedRobot {
  private static MechanismManager mechManager;
  public static MotionMode motionMode = MotionMode.FULL_DRIVE;
  public static FourBar fourBar;
  public static Elevator elevator;
  public static Intake intake;
  public static Vision vision;
  public static SwerveSubsystem swerveDrive;
  public static LightStrip lights;
  private Command autoCommand;

  public static final CommandXboxController driver =
      new CommandXboxController(Constants.RobotMap.DRIVER_PORT);
  public static final CommandXboxController operator =
      new CommandXboxController(Constants.RobotMap.OPERATOR_PORT);

  public static double[] poseValue;
  DoubleArraySubscriber visionPose;

  DoubleArraySubscriber camera2TagPose;

  @Override
  public void robotInit() {
    CameraServer.startAutomaticCapture();
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    visionPose = table.getDoubleArrayTopic("botpose").subscribe(new double[] {});
    camera2TagPose = table.getDoubleArrayTopic("targetpose_cameraspace").subscribe(new double[] {});
    Logger.getInstance().addDataReceiver(new NT4Publisher());
    Logger.getInstance().recordMetadata("GitRevision", Integer.toString(GVersion.GIT_REVISION));
    Logger.getInstance().recordMetadata("GitSHA", GVersion.GIT_SHA);
    Logger.getInstance().recordMetadata("GitDate", GVersion.GIT_DATE);
    Logger.getInstance().recordMetadata("GitBranch", GVersion.GIT_BRANCH);
    Logger.getInstance().recordMetadata("BuildDate", GVersion.BUILD_DATE);

    Logger.getInstance().start();

    fourBar = new FourBar(isSimulation() ? new FourBarIOSim() : new FourBarIOSparks());
    elevator = new Elevator(isSimulation() ? new ElevatorIOSim() : new ElevatorIOSparks());
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

    elevator.setDefaultCommand(
        new InstantCommand(
            () ->
                elevator.setTargetHeight(
                    MathUtil.clamp(
                        elevator.getTargetHeight()
                            + (MathUtil.applyDeadband(
                                    -operator.getRightY(), DriveConstants.K_JOYSTICK_TURN_DEADZONE)
                                / 10),
                        0,
                        50)),
            elevator));

    // lights.setDefaultCommand(LightStrip.Commands.defaultColorPattern());

    mechManager = new MechanismManager();
    autoCommand = new OneToAToThreeToBridge();

    // Driver Controls
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

    driver
        .leftBumper()
        .onTrue(
            new SequentialCommandGroup(
                Elevator.Commands.elevatorCubeFloorIntakeAndWait(),
                new ParallelCommandGroup(
                    Intake.Commands.setWheelVelocityRPM(100),
                    Intake.Commands.setRollerVelocityRPM(100),
                    FourBar.Commands.extend())))
        .onFalse(
            new SequentialCommandGroup(
                Elevator.Commands.elevatorCurrentHeight(),
                new WaitCommand(0.5),
                Intake.Commands.setWheelVelocityRPM(0),
                Intake.Commands.setRollerVelocityRPM(0),
                FourBar.Commands.retract()));

    driver
        .rightTrigger(0.25)
        .onTrue(
            new SequentialCommandGroup(
                Elevator.Commands.elevatorConeFloorTippedIntakeAndWait(),
                new ParallelCommandGroup(
                    Intake.Commands.setWheelVelocityRPM(100),
                    Intake.Commands.setRollerVelocityRPM(100),
                    FourBar.Commands.extend())))
        .onFalse(
            new SequentialCommandGroup(
                Elevator.Commands.elevatorCurrentHeight(),
                new WaitCommand(0.5),
                Intake.Commands.setWheelVelocityRPM(0),
                Intake.Commands.setRollerVelocityRPM(0),
                FourBar.Commands.retract()));

    driver
        .rightBumper()
        .onTrue(
            new SequentialCommandGroup(
                Elevator.Commands.elevatorConeFloorUpIntakeAndWait(),
                new ParallelCommandGroup(
                    Intake.Commands.setWheelVelocityRPM(100),
                    Intake.Commands.setRollerVelocityRPM(100),
                    FourBar.Commands.extend())))
        .onFalse(
            new SequentialCommandGroup(
                Elevator.Commands.elevatorCurrentHeight(),
                new WaitCommand(0.5),
                Intake.Commands.setWheelVelocityRPM(0),
                Intake.Commands.setRollerVelocityRPM(0),
                FourBar.Commands.retract()));

    driver
        .b()
        .onTrue(FourBar.Commands.extend())
        .onFalse(new SequentialCommandGroup(new WaitCommand(0.5), FourBar.Commands.retract()));

    driver
        .y()
        .onTrue(
            new ParallelCommandGroup(
                Intake.Commands.setRollerVelocityRPM(100),
                Intake.Commands.setWheelVelocityRPM(100)))
        .onFalse(
            new SequentialCommandGroup(
                Intake.Commands.setRollerVelocityRPM(0),
                Intake.Commands.setWheelVelocityRPM(0),
                new WaitCommand(0.5),
                FourBar.Commands.retract(),
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
    TimestampedDoubleArray[] fQueue = visionPose.readQueue();
    TimestampedDoubleArray[] cQueue = camera2TagPose.readQueue();

    if (fQueue.length > 0 && cQueue.length > 0) {
      TimestampedDoubleArray fLastCameraReading = fQueue[fQueue.length - 1];
      TimestampedDoubleArray cLastCameraReading = cQueue[cQueue.length - 1];
      swerveDrive.updateVisionPose(fLastCameraReading, cLastCameraReading);
    }
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
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
