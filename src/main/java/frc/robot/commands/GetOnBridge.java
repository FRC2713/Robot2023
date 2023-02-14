package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.MotionHandler.MotionMode;

public class GetOnBridge extends SequentialCommandGroup {
  public GetOnBridge() {
    addCommands(
        new RunCommand(
                () -> {
                  Robot.motionMode = MotionMode.NULL;
                  Robot.swerveDrive.setModuleStates(
                      new SwerveModuleState[] {
                        new SwerveModuleState(-0.7, Rotation2d.fromDegrees(Constants.zero)),
                        new SwerveModuleState(-0.7, Rotation2d.fromDegrees(Constants.zero)),
                        new SwerveModuleState(-0.7, Rotation2d.fromDegrees(Constants.zero)),
                        new SwerveModuleState(-0.7, Rotation2d.fromDegrees(Constants.zero))
                      });
                })
            .until(() -> Robot.swerveDrive.inputs.gyroPitchPosition == 33),
        new RunCommand(
                () -> {
                  Robot.motionMode = MotionMode.NULL;
                  Robot.swerveDrive.setModuleStates(
                      new SwerveModuleState[] {
                        new SwerveModuleState(0.2, Rotation2d.fromDegrees(Constants.zero)),
                        new SwerveModuleState(0.2, Rotation2d.fromDegrees(Constants.zero)),
                        new SwerveModuleState(0.2, Rotation2d.fromDegrees(Constants.zero)),
                        new SwerveModuleState(0.2, Rotation2d.fromDegrees(Constants.zero))
                      });
                })
            .until(() -> Robot.swerveDrive.gyroPitchHasChanged()));
  }
}
