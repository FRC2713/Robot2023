package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.Robot;
import frc.robot.util.MotionHandler.MotionMode;

public class OnBridgeUntilMovement extends SequentialCommandGroup {
  double robotSpeed = 0;
  double backSpeed = 0;

  public OnBridgeUntilMovement(boolean gridside) {

    if ((gridside && DriverStation.getAlliance() == Alliance.Blue)
        || (!gridside && DriverStation.getAlliance() == Alliance.Red)) {
      robotSpeed = 2;
      backSpeed = 0.4;
    } else {
      robotSpeed = -2;
      backSpeed = -0.4;
    }

    addCommands(
        new RunCommand(
                () -> {
                  Robot.motionMode = MotionMode.NULL;
                  Robot.swerveDrive.setModuleStates(
                      DriveConstants.KINEMATICS.toSwerveModuleStates(
                          ChassisSpeeds.fromFieldRelativeSpeeds(
                              robotSpeed, 0, 0, Rotation2d.fromDegrees(0))));
                })
            .until(() -> Robot.swerveDrive.inputs.gyroRollPosition >= 5),
        new InstantCommand(
            () -> {
              Robot.swerveDrive.setModuleStates(
                  DriveConstants.KINEMATICS.toSwerveModuleStates(
                      ChassisSpeeds.fromFieldRelativeSpeeds(
                          backSpeed, 0, 0, Rotation2d.fromDegrees(0))));
            }),
        new WaitCommand(2),
        new InstantCommand(
            () -> {
              Robot.motionMode = MotionMode.LOCKDOWN;
            }));
  }
}
