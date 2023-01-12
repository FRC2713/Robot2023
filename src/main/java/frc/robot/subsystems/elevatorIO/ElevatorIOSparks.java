package frc.robot.subsystems.elevatorIO;

import com.revrobotics.CANSparkMax;

public class ElevatorIOSparks implements ElevatorIO {
  private CANSparkMax left, right;

  @Override
  public void updateInputs(ElevatorInputs inputs) {}

  @Override
  public void setVoltage(double volts) {}
}
