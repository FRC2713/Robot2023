package frc.robot.subsystems.fourBarIO;

import org.littletonrobotics.junction.AutoLog;

public interface FourBarIO {
  @AutoLog
  public static class FourBarInputs {
    public double outputVoltage = 0.0;
    public double angleRads = 0.0;
    public double velocityRadsPerSecond = 0.0;
    public double tempCelcius = 0.0;
  }

  public void updateInputs(FourBarInputs inputs);

  public void setVoltage(double volts);
}
