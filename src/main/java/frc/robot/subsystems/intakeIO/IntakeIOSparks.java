package frc.robot.subsystems.intakeIO;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxAnalogSensor.Mode;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.util.RedHawkUtil;

public class IntakeIOSparks implements IntakeIO {

  private CANSparkMax wheels, rollers;

  public IntakeIOSparks() {
    wheels = new CANSparkMax(Constants.RobotMap.INTAKE_WHEELS_CANID, MotorType.kBrushless);
    rollers = new CANSparkMax(Constants.RobotMap.INTAKE_ROLLERS_CANID, MotorType.kBrushless);
    wheels.restoreFactoryDefaults();
    rollers.restoreFactoryDefaults();

    RedHawkUtil.configureLowTrafficSpark(wheels);
    RedHawkUtil.configureLowTrafficSpark(rollers);

    // analog sensor voltage, analog sensor velocity, analog sensor position
    wheels.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 10);

    wheels.setSmartCurrentLimit(Constants.IntakeConstants.WHEELS_CURRENT_LIMIT);
    rollers.setSmartCurrentLimit(Constants.IntakeConstants.ROLLERS_CURRENT_LIMIT);
    wheels.setIdleMode(IdleMode.kBrake);
    rollers.setIdleMode(IdleMode.kBrake);
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
    inputs.wheelsOutputVoltage = MathUtil.clamp(wheels.getAppliedOutput() * 12, -12.0, 12.0);
    inputs.rollersOutputVoltage = MathUtil.clamp(rollers.getAppliedOutput() * 12, -12.0, 12.0);
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
    inputs.encoderPosition = wheels.getAnalog(Mode.kAbsolute).getPosition();
    inputs.encoderVelocity = wheels.getAnalog(Mode.kAbsolute).getVelocity();
    inputs.encoderVoltage = wheels.getAnalog(Mode.kAbsolute).getVoltage();
  }

  @Override
  public void setVoltageWheels(double volts) {
    wheels.setVoltage(volts);
  }

  @Override
  public void setVoltageRollers(double volts) {
    rollers.setVoltage(volts);
  }

  public void setCurrentLimit(int currentLimit) {
    wheels.setSmartCurrentLimit(currentLimit);
    rollers.setSmartCurrentLimit(currentLimit);
  }
}
