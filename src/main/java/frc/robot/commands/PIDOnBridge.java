package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.Robot;
import frc.robot.util.MotionHandler.MotionMode;
import frc.robot.util.PIDFFController;

public class PIDOnBridge extends SequentialCommandGroup {
  double maxRampAngle = 33;
  double rampSpeed = 0;
  double crawlSpeed = 0;
  PIDFFController controller = new PIDFFController(DriveConstants.K_BRIDGE_CONTROLLER_GAINS);

  public PIDOnBridge(boolean gridside) {
    rampSpeed = gridside ? -0.7 : 0.7;
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
            .until(() -> Robot.swerveDrive.inputs.gyroPitchPosition >= maxRampAngle),
        new RunCommand(
            () -> {
              Robot.motionMode = MotionMode.NULL;
              Robot.swerveDrive.setModuleStates(
                  DriveConstants.KINEMATICS.toSwerveModuleStates(
                      ChassisSpeeds.fromFieldRelativeSpeeds(
                          controller.calculate(Robot.swerveDrive.inputs.gyroPitchPosition, 0),
                          0,
                          0,
                          Rotation2d.fromDegrees(Robot.swerveDrive.inputs.gyroYawPosition))));
            }));
  }
}
