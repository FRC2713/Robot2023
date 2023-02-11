package frc.robot.util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.Constants;

public class PIDFFController extends PIDController {
  private PIDFFGains gains;
  private SimpleMotorFeedforward feedforward;

  public PIDFFController(PIDFFGains gains) {
    super(gains.kP.get(), gains.kI.get(), gains.kD.get());

    this.gains = gains;
    feedforward = gains.createWpilibFeedforward();
    setTolerance(gains.tolerance.get());
  }

  @Override
  public double calculate(double measurement, double setpoint) {
    setSetpoint(setpoint);
    return calculate(measurement);
  }

  @Override
  public double calculate(double measurement) {
    if (Constants.TUNING_MODE) {
      if (gains.hasChanged(this, feedforward)) {
        setP(gains.kP.get());
        setI(gains.kI.get());
        setD(gains.kD.get());
        setTolerance(gains.tolerance.get());
        feedforward = gains.createWpilibFeedforward();
      }
    }

    return super.calculate(measurement) + feedforward.calculate(getSetpoint());
  }
}
