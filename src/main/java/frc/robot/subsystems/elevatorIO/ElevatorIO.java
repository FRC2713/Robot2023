package frc.robot.subsystems.elevatorIO;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorInputs {
    public double outputVoltage = 0.0;
    public double heightInches = 0.0;
    public double velocityInchesPerSecond = 0.0;
    public double tempCelcius = 0.0;

  }

  public void updateInputs(ElevatorInputs inputs);

  public void setVoltage(double volts);
}
