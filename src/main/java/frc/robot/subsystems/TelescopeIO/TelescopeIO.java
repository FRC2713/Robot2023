package frc.robot.subsystems.TelescopeIO;

import org.littletonrobotics.junction.AutoLog;

public interface TelescopeIO {
  @AutoLog
  public static class TelescopeInputs {
    public double outputVoltage = 0.0;
    public double heightInches = 0.0;
    public double velocityInchesPerSecond = 0.0;
    public double tempCelcius = 0.0;
  }

  public void updateInputs(TelescopeInputs inputs);

  public void setVoltage(double volts);
}
