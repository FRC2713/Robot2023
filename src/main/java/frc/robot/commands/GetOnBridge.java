package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.util.MotionHandler.MotionMode;

public class GetOnBridge extends SequentialCommandGroup {
  public GetOnBridge() {
    addCommands(
        new InstantCommand(
            () -> {
              Robot.motionMode = MotionMode.NULL;
              Robot.swerveDrive.setModuleStates(
                  new SwerveModuleState[] {
                    new SwerveModuleState(0.2, Rotation2d.fromDegrees(0)),
                    new SwerveModuleState(0.2, Rotation2d.fromDegrees(-0)),
                    new SwerveModuleState(0.2, Rotation2d.fromDegrees(-0)),
                    new SwerveModuleState(0.2, Rotation2d.fromDegrees(0))
                  });
            }));
  }
}
