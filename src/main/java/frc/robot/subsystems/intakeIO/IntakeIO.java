package frc.robot.subsystems.intakeIO;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  @AutoLog
  public static class IntakeInputs {
    public double topOutputVoltage = 0.0;
    public double bottomOutputVoltage = 0.0;
    public boolean topIsOn = false;
    public boolean bottomIsOn = false;
    public double topVelocityRPM = 0.0;
    public double bottomVelocityRPM = 0.0;
    public double topTempCelcius = 0.0;
    public double bottomTempCelcius = 0.0;
    public double topCurrentAmps = 0.0;
    public double bottomCurrentAmps = 0.0;
    public double topPositionRad = 0.0;
    public double bottomPositionRad = 0.0;
    public double encoderVoltage = 0.0;
    public double encoderPosition = 0.0;
    public double encoderVelocity = 0.0;
  }

  public void setCurrentLimit(int currentLimit);

  public void updateInputs(IntakeInputs inputs);

  public void setTopVoltage(double volts);

  public void setBottomVoltage(double volts);
}
