package frc.robot.subsystems.intakeIO;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  @AutoLog
  public static class IntakeInputs {
    public double wheelsOutputVoltage = 0.0;
    public double rollersOutputVoltage = 0.0;
    public boolean wheelsIsOn = false;
    public boolean rollersIsOn = false;
    public double wheelsVelocityRPM = 0.0;
    public double rollersVelocityRPM = 0.0;
    public double wheelsTempCelcius = 0.0;
    public double rollersTempCelcius = 0.0;
    public double wheelsCurrentAmps = 0.0;
    public double rollersCurrentAmps = 0.0;
    public double wheelsPositionRad = 0.0;
    public double rollersPositionRad = 0.0;
  }

  public void setCurrentLimit(int currentLimit);

  public void updateInputs(IntakeInputs inputs);

  public void setVoltageWheels(double volts);

  public void setVoltageRollers(double volts);
}
