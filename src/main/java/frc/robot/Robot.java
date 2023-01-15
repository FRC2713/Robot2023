// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.CommandHelper;
import frc.robot.subsystems.elevatorIO.Elevator;
import frc.robot.subsystems.elevatorIO.ElevatorIOSim;
import frc.robot.subsystems.swerveIO.SwerveIOPigeon2;
import frc.robot.subsystems.swerveIO.SwerveIOSim;
import frc.robot.subsystems.swerveIO.SwerveSubsystem;
import frc.robot.subsystems.swerveIO.module.SwerveModuleIOSim;
import frc.robot.subsystems.swerveIO.module.SwerveModuleIOSparkMAX;
import frc.robot.util.AutoPath.Autos;
import frc.robot.util.MotionHandler.MotionMode;
import frc.robot.util.RedHawkUtil.ErrHandler;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

public class Robot extends LoggedRobot {
  private Elevator ele;
  public static MotionMode motionMode = MotionMode.FULL_DRIVE;
  public static SwerveSubsystem swerveDrive;
  public static final CommandXboxController driver = new CommandXboxController(Constants.zero);
  private Command autoCommand =
      new SequentialCommandGroup(
          new InstantCommand(
              () -> {
                ele.setTargetHeight(30);
                swerveDrive.resetOdometry(Autos.PART_1.getTrajectory().getInitialHolonomicPose());
              }),
          new WaitUntilCommand(() -> ele.atTargetHeight()),
          new ParallelCommandGroup(
              CommandHelper.stringTrajectoriesTogether(Autos.PART_1.getTrajectory()),
              new InstantCommand(() -> ele.setTargetHeight(0))),
          new ParallelCommandGroup(
              CommandHelper.stringTrajectoriesTogether(Autos.PART_2.getTrajectory()),
              new InstantCommand(() -> ele.setTargetHeight(30))),
          new InstantCommand(() -> ele.setTargetHeight(0)),
          new WaitUntilCommand(() -> ele.atTargetHeight()),
          CommandHelper.stringTrajectoriesTogether(Autos.PART_3.getTrajectory()));

  private Command elevatorTestCommand =
      new SequentialCommandGroup(
          new InstantCommand(
              () -> {
                ele.setTargetHeight(23.5);
              }),
          new WaitUntilCommand(() -> ele.atTargetHeight()),
          new WaitUntilCommand(2),
          new InstantCommand(
              () -> {
                ele.setTargetHeight(35.5);
              }));

  @Override
  public void robotInit() {
    Logger.getInstance().addDataReceiver(new NT4Publisher());
    Logger.getInstance().start();

    this.ele = new Elevator(new ElevatorIOSim());

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

    driver
        .y()
        .onTrue(
            new InstantCommand(
                () -> {
                  motionMode = MotionMode.LOCKDOWN;
                }));
    driver
        .a()
        .onTrue(
            new InstantCommand(
                () -> {
                  motionMode = MotionMode.FULL_DRIVE;
                }));
    driver
        .b()
        .onTrue(
            new InstantCommand(
                () -> {
                  motionMode = MotionMode.SNAPPED_TO_GOAL;
                }));
    driver
        .x()
        .onTrue(
            new InstantCommand(
                () -> {
                  switch (motionMode) {
                    case LOCKDOWN:
                      motionMode = MotionMode.FULL_DRIVE;
                      SmartDashboard.putString("Motion Mode", "Full Drive");
                      break;
                    case FULL_DRIVE:
                      motionMode = MotionMode.SNAPPED_TO_GOAL;
                      SmartDashboard.putString("Motion Mode", "Snapped to Goal");
                      break;
                    case SNAPPED_TO_GOAL:
                      motionMode = MotionMode.LOCKDOWN;
                      SmartDashboard.putString("Motion Mode", "Lockdown");
                      break;
                    default:
                      motionMode = MotionMode.LOCKDOWN;
                  }
                }));
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    ErrHandler.getInstance().log();
  }

  @Override
  public void disabledInit() {
    if (autoCommand != null) {
      autoCommand.cancel();
    }

    if (elevatorTestCommand != null) {
      elevatorTestCommand.cancel();
    }

    Robot.motionMode = MotionMode.LOCKDOWN;
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {

    if (autoCommand != null) {}
    autoCommand.schedule();
    motionMode = MotionMode.TRAJECTORY;

    /*
    if (elevatorTestCommand != null) {
      elevatorTestCommand.schedule();
      motionMode = MotionMode.TRAJECTORY;
    }*/

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

    if (elevatorTestCommand != null) {
      elevatorTestCommand.cancel();
    }

    Robot.motionMode = MotionMode.FULL_DRIVE;
  }

  @Override
  public void teleopPeriodic() {}

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
