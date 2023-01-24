package frc.robot.subsystems.fourBarIO;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUsageId;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;

public class FourBarIOSparks implements FourBarIO {
  private CANSparkMax fourBarOne, fourBarTwo;

  public FourBarIOSparks() {
    fourBarOne = new CANSparkMax(0, MotorType.kBrushless);
    fourBarTwo = new CANSparkMax(1, MotorType.kBrushless);
    fourBarOne.restoreFactoryDefaults();
    fourBarTwo.restoreFactoryDefaults();
    fourBarOne.setSmartCurrentLimit(50);
    fourBarTwo.setSmartCurrentLimit(50);
    fourBarOne.getEncoder().setPositionConversionFactor(Constants.FourBarConstants.FOUR_BAR_ANGLE_CONVERSION);
    fourBarTwo.getEncoder().setPositionConversionFactor(Constants.FourBarConstants.FOUR_BAR_ANGLE_CONVERSION);
    fourBarOne.getEncoder().setVelocityConversionFactor(Constants.FourBarConstants.FOUR_BAR_VELOCITY_CONVERSION_FACTOR);
    fourBarTwo.getEncoder().setVelocityConversionFactor(Constants.FourBarConstants.FOUR_BAR_VELOCITY_CONVERSION_FACTOR);
  }

  @Override
  public void updateInputs(FourBarInputs inputs) {
    inputs.outputVoltage = MathUtil.clamp(fourBarOne.getOutputCurrent(), -12, 12);
    inputs.angleDegrees = fourBarOne.getEncoder().getPosition();
    inputs.velocityDegreesPerSecond = fourBarOne.getEncoder().getVelocity();
    inputs.tempCelcius = fourBarOne.getMotorTemperature();
  }

  @Override
  public void setVoltage(double volts) {
    fourBarOne.setVoltage(volts);
    fourBarTwo.setVoltage(volts);
  }
}