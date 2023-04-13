package frc.robot.commands;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.Robot;
import frc.robot.util.MotionHandler.MotionMode;
import org.littletonrobotics.junction.Logger;

public class PIDOnBridgeExperimental extends SequentialCommandGroup {
  class BangBang {
    double setpoint, tolerance, lastMeasurement;
    SlewRateLimiter limiter;
    public double speed;
    private double prevError = 0;

    public BangBang(double setpoint, double tolerance) {
      this.speed = 0.8;
      this.setpoint = setpoint;
      this.tolerance = tolerance;
      this.limiter = new SlewRateLimiter(speed);

      this.lastMeasurement = 0;
      this.prevError = 0;
    }

    double calculate(double measurement) {
      double currentError = measurement - setpoint * (Robot.slapping ? 1 : -1);
      double rollSpeed = Math.abs(measurement - lastMeasurement);
      if (Math.signum(prevError) != Math.signum(currentError)) {
        speed *= .65;
      }
      lastMeasurement = measurement;
      prevError = currentError;
      
      double out = limiter.calculate(speed);
      Logger.getInstance().recordOutput("PIDBridge/speed", out);
      Logger.getInstance().recordOutput("PIDBridge/currentError", currentError);
      Logger.getInstance().recordOutput("PIDBridge/rollspeed", rollSpeed);
      if (currentError > tolerance && rollSpeed < 0.25) {
        return out;
      } else if (currentError < -tolerance && rollSpeed < 0.25) {
        return -out;
      }
      return 0;
    }
  }

  double maxRampAngle = 12;
  double rampSpeed = 0;
  LinearFilter filter = LinearFilter.singlePoleIIR(0., 0.02);

  public PIDOnBridgeExperimental(boolean gridside) {
    BangBang controller = new BangBang(0, 4.5);
    if ((gridside && DriverStation.getAlliance() == Alliance.Blue)
        || (!gridside && DriverStation.getAlliance() == Alliance.Red)) {
      rampSpeed = 1.75;
    } else {
      rampSpeed = -1.75;
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
        new WaitCommand(0.25),
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
            }));
  }
}
