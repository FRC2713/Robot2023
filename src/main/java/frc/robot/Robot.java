// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.elevatorIO.Elevator;
import frc.robot.subsystems.elevatorIO.ElevatorIOSim;
import frc.robot.subsystems.swerveIO.SwerveIOPigeon2;
import frc.robot.subsystems.swerveIO.SwerveIOSim;
import frc.robot.subsystems.swerveIO.SwerveSubsystem;
import frc.robot.subsystems.swerveIO.module.SwerveModuleIOSim;
import frc.robot.subsystems.swerveIO.module.SwerveModuleIOSparkMAX;
import frc.robot.subsystems.telescopeIO.Telescope;
import frc.robot.subsystems.telescopeIO.TelescopeIOSim;
import frc.robot.util.MotionHandler.MotionMode;
import frc.robot.util.RedHawkUtil.ErrHandler;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

public class Robot extends LoggedRobot {
  private Elevator ele;
  private Telescope tel;
  public static MotionMode motionMode = MotionMode.FULL_DRIVE;
  public static SwerveSubsystem swerveDrive;
  public static final CommandXboxController driver = new CommandXboxController(Constants.zero);

  public static PathPlannerTrajectory traj =
      PathPlanner.loadPath("load4thcargo", PathPlanner.getConstraintsFromPath("load4thcargo"));
  // private Command autoCommand =
  // StringMultipleAutosTogether.stringTrajectoriesTogether(traj);
  private Command autoCommand = new InstantCommand(() -> tel.setTargetExtensionInches(30));

  @Override
  public void robotInit() {
    Logger.getInstance().addDataReceiver(new NT4Publisher());
    Logger.getInstance().start();

    this.ele = new Elevator(new ElevatorIOSim());
    this.tel = new Telescope(new TelescopeIOSim());

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
    /*
     * driver
     * .y()
     * .onTrue(
     * new InstantCommand(
     * () -> {
     * motionMode = MotionMode.LOCKDOWN;
     * }));
     * driver
     * .a()
     * .onTrue(
     * new InstantCommand(
     * () -> {
     * motionMode = MotionMode.FULL_DRIVE;
     * }));
     * driver
     * .b()
     * .onTrue(
     * new InstantCommand(
     * () -> {
     * motionMode = MotionMode.HEADING_CONTROLLER;
     * }));
     *
     */
    driver
        .x()
        .onTrue(
            new InstantCommand(
                () -> {
                  ele.setTargetHeight(30);
                }));

    driver
        .back()
        .onTrue(
            (new InstantCommand(
                () -> {
                  switch (motionMode) {
                    case LOCKDOWN:
                      motionMode = MotionMode.FULL_DRIVE;
                      SmartDashboard.putString("Motion Mode", "Full Drive");
                      break;
                    case FULL_DRIVE:
                      motionMode = MotionMode.HEADING_CONTROLLER;
                      SmartDashboard.putString("Motion Mode", "Heading Controller");
                      break;
                    case HEADING_CONTROLLER:
                      motionMode = MotionMode.LOCKDOWN;
                      SmartDashboard.putString("Motion Mode", "Lockdown");
                      break;
                    default:
                      motionMode = MotionMode.LOCKDOWN;
                  }
                })));
    driver
        .povUp()
        .onTrue(
            (new InstantCommand(
                () -> {
                  motionMode = MotionMode.FULL_DRIVE;
                  SmartDashboard.putString("Motion Mode", "Full Drive");
                })));
    driver
        .povDown()
        .onTrue(
            (new InstantCommand(
                () -> {
                  motionMode = MotionMode.LOCKDOWN;
                  SmartDashboard.putString("Motion Mode", "Lockdown");
                })));
    driver
        .povLeft()
        .onTrue(
            (new InstantCommand(
                () -> {
                  motionMode = MotionMode.HEADING_CONTROLLER;
                  SmartDashboard.putString("Motion Mode", "Heading Controller");
                })));
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
