package frc.robot.subsystems.telescopeIO;

import com.revrobotics.CANSparkMax;

public class TelescopeIOSparks implements TelescopeIO {

  private CANSparkMax left, right;

  @Override
  public void updateInputs(TelescopeInputs inputs) {}

  @Override
  public void setVoltage(double volts) {}
}
