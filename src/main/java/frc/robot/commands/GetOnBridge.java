package frc.robot.commands;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.Robot;
import frc.robot.util.MotionHandler.MotionMode;
import org.littletonrobotics.junction.Logger;

public class GetOnBridge extends SequentialCommandGroup {
  double rampSpeed = 0;
  double crawlSpeed = 0;
  LinearFilter filter = LinearFilter.highPass(0.04, 0.02);

  public GetOnBridge(boolean gridside) {
    rampSpeed = (2) * (gridside ? 1 : -1);
    crawlSpeed = 0.4 * (gridside ? 1 : -1);
    addCommands(
        // new RunCommand(
        //         () -> {
        //           Robot.motionMode = MotionMode.NULL;
        //           Robot.swerveDrive.setModuleStates(
        //               DriveConstants.KINEMATICS.toSwerveModuleStates(
        //                   ChassisSpeeds.fromFieldRelativeSpeeds(
        //                       crawlSpeed,
        //                       0,
        //                       0,
        //
        // Rotation2d.fromDegrees(Robot.swerveDrive.inputs.gyroYawPosition))));
        //         })
        //     .until(() -> Robot.swerveDrive.inputs.gyroRollPosition >= 5),
        new RunCommand(
                () -> {
                  var value = filter.calculate(Robot.swerveDrive.inputs.gyroRollPosition);
                  Logger.getInstance().recordOutput("Bridge value", value);
                  Robot.motionMode = MotionMode.NULL;
                  Robot.swerveDrive.setModuleStates(
                      DriveConstants.KINEMATICS.toSwerveModuleStates(
                          ChassisSpeeds.fromFieldRelativeSpeeds(
                              rampSpeed,
                              0,
                              0,
                              Rotation2d.fromDegrees(Robot.swerveDrive.inputs.gyroYawPosition))));
                })
            .until(() -> Robot.swerveDrive.inputs.gyroRollPosition >= 1),
        new RunCommand(
                () -> {
                  var value = filter.calculate(Robot.swerveDrive.inputs.gyroRollPosition);
                  Logger.getInstance().recordOutput("Bridge value", value);

                  Robot.motionMode = MotionMode.NULL;
                  Robot.swerveDrive.setModuleStates(
                      DriveConstants.KINEMATICS.toSwerveModuleStates(
                          ChassisSpeeds.fromFieldRelativeSpeeds(
                              crawlSpeed
                                  * Math.signum(Robot.swerveDrive.inputs.gyroRollPosition)
                                  * Math.signum(crawlSpeed),
                              0,
                              0,
                              Rotation2d.fromDegrees(Robot.swerveDrive.inputs.gyroYawPosition))));
                })
            .until(() -> Math.abs(Robot.swerveDrive.inputs.gyroRollPosition) <= 4),
        new InstantCommand(
            () ->
                Robot.swerveDrive.setModuleStates(
                    DriveConstants.KINEMATICS.toSwerveModuleStates(
                        ChassisSpeeds.fromFieldRelativeSpeeds(
                            0,
                            0,
                            0,
                            Rotation2d.fromDegrees(Robot.swerveDrive.inputs.gyroYawPosition)))))
        // new InstantCommand(() -> Robot.motionMode = MotionMode.LOCKDOWN)
        );
  }
}
