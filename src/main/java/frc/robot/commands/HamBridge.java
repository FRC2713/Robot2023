package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.Robot;
import frc.robot.util.MotionHandler.MotionMode;
import org.littletonrobotics.junction.Logger;

public class HamBridge extends SequentialCommandGroup {
  private PIDController balancePID = new PIDController(2, 0, 0);

  private Command hamBridge =
      new RunCommand(
          () -> {
            double pidOutput =
                    balancePID.calculate(Robot.swerveDrive.filteredRollVal, 0);
            Logger.getInstance().recordOutput("HamBridgePIDOutput", pidOutput);
            pidOutput = MathUtil.clamp(pidOutput, -1.0, 1.0);
            Robot.swerveDrive.setModuleStates(
                DriveConstants.KINEMATICS.toSwerveModuleStates(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        pidOutput,
                        0,
                        0,
                        Rotation2d.fromDegrees(Robot.swerveDrive.inputs.gyroYawPosition))));
          });

  public HamBridge() {
    balancePID.setTolerance(Units.radiansToDegrees(0.005));
    addCommands(
        new RunCommand(
                () -> {
                  Robot.motionMode = MotionMode.NULL;
                  Robot.swerveDrive.setModuleStates(
                      DriveConstants.KINEMATICS.toSwerveModuleStates(
                          ChassisSpeeds.fromFieldRelativeSpeeds(
                              2,
                              0,
                              0,
                              Rotation2d.fromDegrees(Robot.swerveDrive.inputs.gyroYawPosition))));
                })
            .until(() -> Math.abs(Robot.swerveDrive.filteredRollVal) >= 14),
        hamBridge);
  }
}
