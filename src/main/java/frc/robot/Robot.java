// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.OTF.Dynamic;
import frc.robot.commands.OTF.GoClosestGrid;
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
import frc.robot.util.MechanismManager;
import frc.robot.util.MotionHandler.MotionMode;
import frc.robot.util.RedHawkUtil;
import frc.robot.util.RedHawkUtil.ErrHandler;
import frc.robot.util.SwerveHeadingController;
import frc.robot.util.TrajectoryController;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

public class Robot extends LoggedRobot {
  public static FourBar four;
  public static Elevator ele;
  public static Intake intake;
  public static Vision vis;
  public static double[] poseValue;
  DoubleArraySubscriber visionPose;
  private static MechanismManager mechManager;
  public static MotionMode motionMode = MotionMode.FULL_DRIVE;
  public static SwerveSubsystem swerveDrive;
  public static final CommandXboxController driver = new CommandXboxController(Constants.zero);
  private Command autoCommand;

  @Override
  public void robotInit() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    visionPose = table.getDoubleArrayTopic("botpose").subscribe(new double[] {});
    Logger.getInstance().addDataReceiver(new NT4Publisher());
    Logger.getInstance().recordMetadata("GitRevision", Integer.toString(GVersion.GIT_REVISION));
    Logger.getInstance().recordMetadata("GitSHA", GVersion.GIT_SHA);
    Logger.getInstance().recordMetadata("GitDate", GVersion.GIT_DATE);
    Logger.getInstance().recordMetadata("GitBranch", GVersion.GIT_BRANCH);
    Logger.getInstance().recordMetadata("BuildDate", GVersion.BUILD_DATE);

    Logger.getInstance().start();

    four = new FourBar(isSimulation() ? new FourBarIOSim() : new FourBarIOSparks());
    mechManager = new MechanismManager();
    ele = new Elevator(isSimulation() ? new ElevatorIOSim() : new ElevatorIOSparks());
    intake = new Intake(isSimulation() ? new IntakeIOSim() : new IntakeIOSparks());
    vis = new Vision(new VisionIOSim());

    Robot.swerveDrive =
        Robot.isReal()
            ? new SwerveSubsystem(
                new SwerveIOPigeon2(),
                new SwerveModuleIOSparkMAX(Constants.DriveConstants.frontLeft),
                new SwerveModuleIOSparkMAX(Constants.DriveConstants.frontRight),
                new SwerveModuleIOSparkMAX(Constants.DriveConstants.backLeft),
                new SwerveModuleIOSparkMAX(Constants.DriveConstants.backRight))
            : new SwerveSubsystem(
                new SwerveIOSim(),
                new SwerveModuleIOSim(Constants.DriveConstants.frontLeft),
                new SwerveModuleIOSim(Constants.DriveConstants.frontRight),
                new SwerveModuleIOSim(Constants.DriveConstants.backLeft),
                new SwerveModuleIOSim(Constants.DriveConstants.backRight));

    // autoCommand = new SequentialCommandGroup(
    // new InstantCommand(
    // () -> {
    // Robot.swerveDrive.resetOdometry(
    // ReflectedTransform.reflectiveTransformTrajectory(
    // PathPlanner.loadPath(
    // "goto1stcargo",
    // PathPlanner.getConstraintsFromPath("goto1stcargo")))
    // .getInitialHolonomicPose());
    // }),
    // SwerveSubsystem.Commands.stringTrajectoriesTogether(
    // ReflectedTransform.reflectiveTransformTrajectory(
    // PathPlanner.loadPath(
    // "goto1stcargo", PathPlanner.getConstraintsFromPath("goto1stcargo")))),
    // new WaitCommand(10),
    // new GoGridOne());

    driver
        .x()
        .onTrue(
            new InstantCommand(
                () -> {
                  ele.setTargetHeight(0);
                }));
    driver
        .y()
        .onTrue(
            new InstantCommand(
                () -> {
                  ele.setTargetHeight(30);
                }));

    driver
        .back()
        .onTrue(
            new InstantCommand(
                () -> {
                  // ele.setTargetHeight(30);
                }));
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
            new InstantCommand(
                () -> {
                  motionMode = MotionMode.TRAJECTORY;
                  TrajectoryController.getInstance().changePath(new Dynamic().getTrajectory());
                },
                swerveDrive))
        .onFalse(
            new InstantCommand(
                () -> {
                  motionMode = MotionMode.FULL_DRIVE;
                },
                swerveDrive));
    GoClosestGrid go = new GoClosestGrid();
    driver
        .rightBumper()
        .whileTrue(
            new RepeatCommand(
                new InstantCommand(
                    () -> {
                      motionMode = MotionMode.TRAJECTORY;
                      TrajectoryController.getInstance()
                          .changePath(go.regenerateTrajectory().getTrajectory());
                      // TrajectoryController.getInstance().changePath(go.getTrajectory());
                    },
                    swerveDrive)))
        .onFalse(
            new InstantCommand(
                () -> {
                  motionMode = MotionMode.FULL_DRIVE;
                },
                swerveDrive));
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    ErrHandler.getInstance().log();
    mechManager.periodic();
    if (Math.abs(driver.getRightX()) > 0.25) {
      motionMode = MotionMode.FULL_DRIVE;
    }
    Logger.getInstance()
        .recordOutput(
            "SwerveValues",
            new double[] {
              RedHawkUtil.Pose2dToTranslation2d(Robot.swerveDrive.getOdometry().getPoseMeters())
                  .getX(),
              RedHawkUtil.Pose2dToTranslation2d(Robot.swerveDrive.getOdometry().getPoseMeters())
                  .getY(),
              Robot.swerveDrive.inputs.gyroYawPosition,
              Robot.swerveDrive.getRegularPose().getRotation().getDegrees()
            });
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
    // four.setAngleDeg(20);
    // ele.setTargetHeight(30);
    if (autoCommand != null) {
      autoCommand.schedule();
    }
    motionMode = MotionMode.TRAJECTORY;
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
