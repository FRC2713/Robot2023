package frc.robot.commands;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.Robot;
import frc.robot.util.MotionHandler.MotionMode;

public class PIDOnBridge extends SequentialCommandGroup {
  class BangBang {
    double setpoint, tolerance;
    SlewRateLimiter limiter;
    final double speed = 0.33;

    public BangBang(double setpoint, double tolerance) {
      this.setpoint = setpoint;
      this.tolerance = tolerance;
      limiter = new SlewRateLimiter(speed);
    }

    double calculate(double measurement) {
      var out = limiter.calculate(speed);
      if (Math.abs(measurement - setpoint) < tolerance) {
        return 0;
      }
      if (measurement < setpoint) {
        return out;
      }
      return -out;
    }
  }

  double maxRampAngle = 10;
  double rampSpeed = 0;
  double crawlSpeed = 0;
  BangBang controller = new BangBang(0, 4.5);
  LinearFilter filter = LinearFilter.singlePoleIIR(0., 0.02);

  public PIDOnBridge(boolean gridside) {
    rampSpeed = gridside ? 1.1 : -1.1;
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
            .until(() -> (Robot.swerveDrive.filteredRollVal) >= maxRampAngle),
        new RunCommand(
            () -> {
              Robot.motionMode = MotionMode.NULL;
              Robot.swerveDrive.setModuleStates(
                  DriveConstants.KINEMATICS.toSwerveModuleStates(
                      ChassisSpeeds.fromFieldRelativeSpeeds(
                          controller.calculate(Robot.swerveDrive.filteredRollVal) * -1,
                          0,
                          0,
                          Rotation2d.fromDegrees(Robot.swerveDrive.inputs.gyroYawPosition))));
            }),
        new InstantCommand(
            () ->
                Robot.swerveDrive.setModuleStates(
                    DriveConstants.KINEMATICS.toSwerveModuleStates(
                        ChassisSpeeds.fromFieldRelativeSpeeds(
                            0,
                            0,
                            0,
                            Rotation2d.fromDegrees(Robot.swerveDrive.inputs.gyroYawPosition))))));
  }
}
