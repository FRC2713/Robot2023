package frc.robot.subsystems.elevatorIO;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorInputs {
    public double outputVoltageLeft = 0.0;
    public double heightInchesLeft = 0.0;
    public double velocityInchesPerSecondLeft = 0.0;
    public double tempCelsiusLeft = 0.0;
    public double currentDrawAmpsLeft = 0.0;
    public double outputVoltageRight = 0.0;
    public double heightInchesRight = 0.0;
    public double velocityInchesPerSecondRight = 0.0;
    public double tempCelsiusRight = 0.0;
    public double currentDrawAmpsRight = 0.0;
  }

  public void updateInputs(ElevatorInputs inputs);

  public void resetEncoders();

  public boolean shouldApplyFF();

  public void setPIDFF();

  /***
   * Must be called in periodic for FeedForward to properly calculate
   */
  public void updatePID(double heightInchesRight, double setpoint, double ffVolts);
}
