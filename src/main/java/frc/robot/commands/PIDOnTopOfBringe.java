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
import org.littletonrobotics.junction.Logger;

public class PIDOnTopOfBringe extends SequentialCommandGroup {
  class BangBang {
    double setpoint, tolerance, lastMeasurement;
    SlewRateLimiter limiter;
    public double speed;
    private double prevError = 0;

    private double decayRate = 0.65;
    private DriverStation.Alliance alliance;

    public BangBang(double setpoint, double tolerance) {
      this.init();
      this.setpoint = setpoint;
      this.tolerance = tolerance;
      this.limiter = new SlewRateLimiter(this.speed);
    }

    public void init() {
      this.speed = 0.5;
      this.lastMeasurement = 0;
      this.prevError = 0;
      this.alliance = DriverStation.getAlliance();
    }

    double calculate(double measurement) {
      //                                              Robot.slapping
      double currentError = measurement - setpoint * (false ? 1 : -1);
      double rollSpeed = Math.abs(measurement - lastMeasurement);
      if ((Math.signum(prevError) != Math.signum(currentError))) {
        speed *= decayRate;
      }
      lastMeasurement = measurement;
      prevError = currentError;

      double out = limiter.calculate(speed);
      Logger.getInstance().recordOutput("PIDBridge/currentError", currentError);
      Logger.getInstance().recordOutput("PIDBridge/rollspeed", rollSpeed);
      if (currentError > tolerance && rollSpeed < 0.25) {
        out = this.alliance == Alliance.Red ? out : -out;
      } else if (currentError < -tolerance && rollSpeed < 0.25) {
        out = this.alliance == Alliance.Red ? -out : out;
      } else {
        out = 0;
      }
      Logger.getInstance().recordOutput("PIDBridge/speed", out);

      return out;
    }
  }

  double maxRampAngle = 12;
  LinearFilter filter = LinearFilter.singlePoleIIR(0., 0.02);

  public PIDOnTopOfBringe(boolean gridside) {
    BangBang controller = new BangBang(0, 4.5);

    addCommands(
        new InstantCommand(
            () -> {
              controller.init();
            }),
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
