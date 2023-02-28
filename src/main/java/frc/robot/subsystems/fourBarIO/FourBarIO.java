package frc.robot.subsystems.fourBarIO;

import org.littletonrobotics.junction.AutoLog;

public interface FourBarIO {
  @AutoLog
  public static class FourBarInputs {
    public double outputVoltage = 0.0;

    public double angleDegreesOne = 0.0;
    public double angleDegreesTwo = 0.0;
    public double angleDegreesRange = 0.0;

    public double velocityDegreesPerSecondOne = 0.0;
    public double velocityDegreesPerSecondTwo = 0.0;
    public double velocityDegreesPerSecondRange = 0.0;

    public double tempCelciusOne = 0.0;
    public double tempCelciusTwo = 0.0;

    public double currentDrawOne = 0.0;
    public double currentDrawTwo = 0.0;

    public double absoluteEncoderVolts = 0.0;
  }

  public void updateInputs(FourBarInputs inputs);

  public void setVoltage(double volts);
}
