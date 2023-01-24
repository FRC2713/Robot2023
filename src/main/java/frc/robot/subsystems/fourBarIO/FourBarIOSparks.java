package frc.robot.subsystems.fourBarIO;

import com.revrobotics.CANSparkMax;

public class FourBarIOSparks implements FourBarIO {
  private CANSparkMax fourBarOne, fourBarTwo;

  @Override
  public void updateInputs(FourBarInputs inputs) {}

  @Override
  public void setVoltage(double volts) {}
}
