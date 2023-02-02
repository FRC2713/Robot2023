package frc.robot.subsystems.fourBarIO;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;

public class FourBarIOSparks implements FourBarIO {
  private CANSparkMax fourBarOne, fourBarTwo;

  public FourBarIOSparks() {
    fourBarOne = new CANSparkMax(Constants.RobotMap.FOURBAR_ONE_CANID, MotorType.kBrushless);
    fourBarTwo = new CANSparkMax(Constants.RobotMap.FOURBAR_TWO_CANID, MotorType.kBrushless);
    fourBarOne.restoreFactoryDefaults();
    fourBarTwo.restoreFactoryDefaults();
    fourBarOne.setInverted(true); // subject to change
    fourBarTwo.setInverted(true); // subject to change
    fourBarOne.setSmartCurrentLimit(Constants.FourBarConstants.FOUR_BAR_CURRENT_LIMIT);
    fourBarTwo.setSmartCurrentLimit(Constants.FourBarConstants.FOUR_BAR_CURRENT_LIMIT);
    fourBarOne
        .getEncoder()
        .setPositionConversionFactor(Constants.FourBarConstants.FOUR_BAR_ANGLE_CONVERSION);
    fourBarTwo
        .getEncoder()
        .setPositionConversionFactor(Constants.FourBarConstants.FOUR_BAR_ANGLE_CONVERSION);
    fourBarOne
        .getEncoder()
        .setVelocityConversionFactor(
            Constants.FourBarConstants.FOUR_BAR_VELOCITY_CONVERSION_FACTOR);
    fourBarTwo
        .getEncoder()
        .setVelocityConversionFactor(
            Constants.FourBarConstants.FOUR_BAR_VELOCITY_CONVERSION_FACTOR);
  }

  @Override
  public void updateInputs(FourBarInputs inputs) {
    inputs.outputVoltage = MathUtil.clamp(fourBarOne.getOutputCurrent(), -12, 12);
    inputs.angleDegrees = fourBarOne.getEncoder().getPosition();
    inputs.velocityDegreesPerSecond = fourBarOne.getEncoder().getVelocity();
    inputs.tempCelcius = fourBarOne.getMotorTemperature();
    inputs.currentDraw = fourBarOne.getOutputCurrent();
  }

  @Override
  public void setVoltage(double volts) {
    fourBarOne.setVoltage(volts);
    fourBarTwo.setVoltage(volts);
  }
}
