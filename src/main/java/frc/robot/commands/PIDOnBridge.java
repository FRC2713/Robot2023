package frc.robot.commands;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
    public double speed = 0.75;
    private double prevError = 0;

    public BangBang(double setpoint, double tolerance) {
      this.setpoint = setpoint;
      this.tolerance = tolerance;
      limiter = new SlewRateLimiter(speed);
    }

    double calculate(double measurement) {
      double currentError = measurement - setpoint;
      if (Math.signum(prevError) != Math.signum(currentError)) {
        speed *= .9;
      }
      var out = limiter.calculate(speed);
      if (currentError > tolerance) {
        prevError = currentError;
        return -out;
      } else if (currentError < -tolerance) {
        prevError = currentError;
        return out;
      }
      return 0;
    }
  }

  double maxRampAngle = 14;
  double rampSpeed = 0;
  double crawlSpeed = 0;
  BangBang controller = new BangBang(0, 4.5);
  LinearFilter filter = LinearFilter.singlePoleIIR(0., 0.02);

  public PIDOnBridge(boolean gridside) {
    if ((gridside && DriverStation.getAlliance() == Alliance.Blue)
        || (!gridside && DriverStation.getAlliance() == Alliance.Red)) {
      rampSpeed = 2;
    } else {
      rampSpeed = -2;
    }

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
            .until(() -> Math.abs(Robot.swerveDrive.filteredRollVal) >= maxRampAngle),
        new RunCommand(
            () -> {
              Robot.motionMode = MotionMode.NULL;
              Robot.swerveDrive.setModuleStates(
                  DriveConstants.KINEMATICS.toSwerveModuleStates(
                      ChassisSpeeds.fromFieldRelativeSpeeds(
                          controller.calculate(Robot.swerveDrive.filteredRollVal),
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
