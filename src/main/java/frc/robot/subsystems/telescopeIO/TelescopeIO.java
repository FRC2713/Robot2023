package frc.robot.subsystems.telescopeIO;

import org.littletonrobotics.junction.AutoLog;

public interface TelescopeIO {
  @AutoLog
  public static class TelescopeInputs {

    public double outputVoltage = 0.0;

    public double extendedInches = 0.0;

    public double velocityInchesPerSecond = 0.0;

    public double tempCelsius = 0.0;
  }

  public void updateInputs(TelescopeInputs inputs);

  public void setVoltage(double volts);
}
