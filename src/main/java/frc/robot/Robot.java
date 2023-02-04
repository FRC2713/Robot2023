// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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

import static frc.robot.subsystems.LightStrip.Pattern.Purple;
import static frc.robot.subsystems.LightStrip.Pattern.Yellow;

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

  public static final CommandXboxController driver = new CommandXboxController(Constants.RobotMap.DRIVER_PORT);
  public static final CommandXboxController awp = new CommandXboxController(Constants.RobotMap.AWP_PORT);

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
                new SwerveModuleIOSim(Constants.DriveConstants.frontLeft),
                new SwerveModuleIOSim(Constants.DriveConstants.frontRight),
                new SwerveModuleIOSim(Constants.DriveConstants.backLeft),
                new SwerveModuleIOSim(Constants.DriveConstants.backRight))
            : new SwerveSubsystem(
                new SwerveIOPigeon2(),
                new SwerveModuleIOSparkMAX(Constants.DriveConstants.frontLeft),
                new SwerveModuleIOSparkMAX(Constants.DriveConstants.frontRight),
                new SwerveModuleIOSparkMAX(Constants.DriveConstants.backLeft),
                new SwerveModuleIOSparkMAX(Constants.DriveConstants.backRight));

    mechManager = new MechanismManager();
    autoCommand = new OneToAToThreeToBridge();

    //Driver Controls
   driver.povUp().onTrue(new InstantCommand(() -> {
      motionMode = MotionMode.HEADING_CONTROLLER;
      SwerveHeadingController.getInstance().setSetpoint(Rotation2d.fromDegrees(0));
    }));

    driver.povLeft().onTrue(new InstantCommand(() -> {
      motionMode = MotionMode.HEADING_CONTROLLER;
      SwerveHeadingController.getInstance().setSetpoint(Rotation2d.fromDegrees(90));
    }));

    driver.povDown().onTrue(new InstantCommand(() -> {
      motionMode = MotionMode.HEADING_CONTROLLER;
      SwerveHeadingController.getInstance().setSetpoint(Rotation2d.fromDegrees(180));
    }));

    driver.povRight().onTrue(new InstantCommand(() -> {
      motionMode = MotionMode.HEADING_CONTROLLER;
      SwerveHeadingController.getInstance().setSetpoint(Rotation2d.fromDegrees(270));
    }));

    driver.leftBumper().whileTrue(new ParallelCommandGroup(
        Elevator.Commands.elevatorCubeFloorIntake(),
        //Intake.Commands.intakeRollersOn();
        //Intake.Commands.intakeWheelsOn();
        FourBar.Commands.cmdSetAngleDeg(Constants.DOUBLE_PLACEHOLDER)
    ));

    //Operator Buttons
    awp.leftBumper().and(awp.y()).onTrue(new ParallelCommandGroup(
        Elevator.Commands.elevatorConeHighScore(),
        FourBar.Commands.cmdExtend()
    ));
    awp.leftBumper().and(awp.b()).onTrue(new ParallelCommandGroup(
      Elevator.Commands.elevatorConeMidScore(),
      FourBar.Commands.cmdExtend()
    ));
    awp.leftBumper().and(awp.a()).onTrue(new ParallelCommandGroup(
      Elevator.Commands.elevatorConeLowScore(),
      FourBar.Commands.cmdExtend()
    ));
    awp.rightBumper().and(awp.y()).onTrue(new ParallelCommandGroup(
      Elevator.Commands.elevatorCubeHighScore(),
      FourBar.Commands.cmdExtend()
    ));
    awp.rightBumper().and(awp.b()).onTrue(new ParallelCommandGroup(
      Elevator.Commands.elevatorCubeMidScore(),
      FourBar.Commands.cmdExtend()
    ));
    awp.rightBumper().and(awp.a()).onTrue(new ParallelCommandGroup(
      Elevator.Commands.elevatorCubeLowScore(),
      FourBar.Commands.cmdExtend()
    ));
    awp.leftTrigger(0.25).onTrue(LightStrip.Commands.setColorPattern(Yellow));
    awp.rightTrigger(0.25).onTrue(LightStrip.Commands.setColorPattern(Purple));
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    ErrHandler.getInstance().log();
    mechManager.periodic();
    if (Math.abs(driver.getRightX()) > 0.25) {
      motionMode = MotionMode.FULL_DRIVE;
    }
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
    TimestampedDoubleArray[] queue = visionPose.readQueue();

    if (queue.length > 0) {
      TimestampedDoubleArray lastCameraReading = queue[queue.length - 1];
      swerveDrive.updateVisionPose(lastCameraReading);
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
