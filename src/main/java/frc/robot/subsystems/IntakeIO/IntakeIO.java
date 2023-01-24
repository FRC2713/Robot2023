package frc.robot.subsystems.IntakeIO;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  @AutoLog
  public static class IntakeInputs {
    public double outputVoltage = 0.0;
    public boolean isOn = false;
    public double velocityRadsPerSecond = 0.0;
    public double tempCelcius = 0.0;
    public double currentAmps = 0.0;
    public double positionRad = 0.0;
  }

  public void updateInputs(IntakeInputs inputs);

  public void setVoltage(double volts);
}
