package frc.robot.subsystems.intakeIO;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class IntakeIOSparks implements IntakeIO {

  private CANSparkMax wheels, rollers;

  public IntakeIOSparks() {
    wheels = new CANSparkMax(0, MotorType.kBrushless);
    rollers = new CANSparkMax(1, MotorType.kBrushless);
    wheels.restoreFactoryDefaults();
    rollers.restoreFactoryDefaults();
    wheels.setSmartCurrentLimit(Constants.IntakeConstants.WHEELS_CURRENT_LIMIT);
    rollers.setSmartCurrentLimit(Constants.IntakeConstants.ROLLERS_CURRENT_LIMIT);
    wheels
        .getEncoder()
        .setPositionConversionFactor(Constants.IntakeConstants.WHEELS_POSITION_CONVERSION_FACTOR);
    rollers
        .getEncoder()
        .setPositionConversionFactor(Constants.IntakeConstants.ROLLERS_POSITION_CONVERSION_FACTOR);
    wheels
        .getEncoder()
        .setVelocityConversionFactor(Constants.IntakeConstants.WHEELS_VELOCITY_CONVERSION_FACTOR);
    rollers
        .getEncoder()
        .setVelocityConversionFactor(Constants.IntakeConstants.ROLLERS_VELOCITY_CONVERSION_FACTOR);
  }

  @Override
  public void updateInputs(IntakeInputs inputs) {
    inputs.wheelsOutputVoltage = MathUtil.clamp(wheels.getOutputCurrent(), -12.0, 12.0);
    inputs.rollersOutputVoltage = MathUtil.clamp(rollers.getOutputCurrent(), -12.0, 12.0);
    inputs.wheelsIsOn =
        Units.rotationsPerMinuteToRadiansPerSecond(wheels.getEncoder().getVelocity()) > 0.005;
    inputs.rollersIsOn =
        Units.rotationsPerMinuteToRadiansPerSecond(rollers.getEncoder().getVelocity()) > 0.005;
    inputs.wheelsVelocityRPM = wheels.getEncoder().getVelocity();
    inputs.rollersVelocityRPM = rollers.getEncoder().getVelocity();
    inputs.wheelsTempCelcius = wheels.getMotorTemperature();
    inputs.rollersTempCelcius = rollers.getMotorTemperature();
    inputs.wheelsCurrentAmps = wheels.getOutputCurrent();
    inputs.rollersCurrentAmps = rollers.getOutputCurrent();
    inputs.wheelsPositionRad =
        Units.rotationsPerMinuteToRadiansPerSecond(wheels.getEncoder().getPosition());
    inputs.rollersPositionRad =
        Units.rotationsPerMinuteToRadiansPerSecond(rollers.getEncoder().getPosition());
  }

  @Override
  public void setVoltage(double volts) {
    wheels.setVoltage(volts);
    rollers.setVoltage(volts);
  }
}
