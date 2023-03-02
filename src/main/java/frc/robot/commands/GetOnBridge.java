package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.Robot;
import frc.robot.util.MotionHandler.MotionMode;

public class GetOnBridge extends SequentialCommandGroup {
  double rampSpeed = 0;
  double crawlSpeed = 0;

  public GetOnBridge(boolean gridside) {
    rampSpeed = gridside ? 0.7 : -0.7;
    crawlSpeed = gridside ? 0.2 : -0.2;
    addCommands(
        new RunCommand(
                () -> {
                  Robot.motionMode = MotionMode.NULL;
                  Robot.swerveDrive.setModuleStates(
                      DriveConstants.KINEMATICS.toSwerveModuleStates(
                          ChassisSpeeds.fromFieldRelativeSpeeds(
                              rampSpeed,
                              0,
                              0,
                              Rotation2d.fromDegrees(Robot.swerveDrive.inputs.gyroYawPosition))));
                })
            .until(() -> Robot.swerveDrive.inputs.gyroPitchPosition >= 33),
        new RunCommand(
                () -> {
                  Robot.motionMode = MotionMode.NULL;
                  Robot.swerveDrive.setModuleStates(
                      DriveConstants.KINEMATICS.toSwerveModuleStates(
                          ChassisSpeeds.fromFieldRelativeSpeeds(
                              crawlSpeed,
                              0,
                              0,
                              Rotation2d.fromDegrees(Robot.swerveDrive.inputs.gyroYawPosition))));
                })
            .until(() -> Robot.swerveDrive.gyroPitchHasChanged()));
  }
}
