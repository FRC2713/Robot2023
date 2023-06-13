package frc.robot.subsystems.slapperIO;

import org.littletonrobotics.junction.AutoLog;

public interface SlapperIO {

  @AutoLog
  public class SlapperInputs {
    public double outputVoltage = 0.0;
    public boolean isOn = false;
    public double velocityRPM = 0.0;
    public double tempCelcius = 0.0;
    public double currentAmps = 0.0;
    public double positionDeg = 0.0;
    public double encoderVoltage = 0.0;
    public double encoderPosition = 0.0;
    public double encoderVelocity = 0.0;
  }

  public void setCurrentLimit(int currentLimit);

  public void updateInputs(SlapperInputs inputs);

  public void setVoltage(double volts);
}
