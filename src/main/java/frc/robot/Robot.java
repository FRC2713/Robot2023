// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

public class Robot extends LoggedRobot {
private Elevator ele;
private Command autoCommand = new StringTwoAutosTogether();
  public static MotionMode motionMode = MotionMode.FULL_DRIVE;
  public static SwerveSubsystem swerveDrive;
  public static final XboxController driver = new XboxController(Constants.zero);
  public static PathPlannerTrajectory traj1 =
      PathPlanner.loadPath("loopdeloop", PathPlanner.getConstraintsFromPath("loopdeloop"));
  public static PathPlannerTrajectory traj2 =
      PathPlanner.loadPath("two!", PathPlanner.getConstraintsFromPath("two!"));

  @Override
  public void robotInit() {
    Logger.getInstance().addDataReceiver(new NT4Publisher());
    Logger.getInstance().start();

this.ele = new Elevator(new ElevatorIOSim());
    m_autonomousCommand =
        new SequentialCommandGroup(
            new InstantCommand(
                () -> {
                  ele.setTargetHeight(30.0);
                }),
            new WaitCommand(5.0),
            new InstantCommand(
                () -> {
                  ele.setTargetHeight(15.0);
                }));
    // repeat :)
    // new SequentialCommandGroup
    // (
    //   new InstantCommand(
    //     () -> {
    //       ele.setTargetHeight(30.0);
    //     }),
    //     new WaitCommand(5.0),
    //     new InstantCommand(
    //       () -> {
    //         ele.setTargetHeight(15.0);
    //       }),
    //       new WaitCommand(5.0)
    // ).repeatedly();
    
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

    new JoystickButton(driver, XboxController.Button.kY.value)
        .onTrue(
            new InstantCommand(
                () -> {
                  motionMode = MotionMode.LOCKDOWN;
                }));

    new JoystickButton(driver, XboxController.Button.kA.value)
        .onTrue(
            new InstantCommand(
                () -> {
                  motionMode = MotionMode.FULL_DRIVE;
                }));

    new JoystickButton(driver, XboxController.Button.kB.value)
        .onTrue(
            new InstantCommand(
                () -> {
                  motionMode = MotionMode.HEADING_CONTROLLER;
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
