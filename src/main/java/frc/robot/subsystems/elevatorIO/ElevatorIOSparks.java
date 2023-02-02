package frc.robot.subsystems.elevatorIO;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class ElevatorIOSparks implements ElevatorIO {
  private CANSparkMax left, right;

  public ElevatorIOSparks() {
    left = new CANSparkMax(0, MotorType.kBrushless);
    right = new CANSparkMax(1, MotorType.kBrushless);
    left.setIdleMode(CANSparkMax.IdleMode.kBrake);
    right.setIdleMode(CANSparkMax.IdleMode.kBrake);
    left.restoreFactoryDefaults();
    right.restoreFactoryDefaults();
    left.setInverted(true);
    right.setInverted(false); // might be reversed, idk
    left.setSmartCurrentLimit(Constants.ElevatorConstants.ELEVATOR_CURRENT_LIMIT);
    right.setSmartCurrentLimit(Constants.ElevatorConstants.ELEVATOR_CURRENT_LIMIT);
    left.getEncoder()
        .setPositionConversionFactor(
            Constants.ElevatorConstants.ELEVATOR_POSITION_CONVERSION_FACTOR);
    right
        .getEncoder()
        .setPositionConversionFactor(
            Constants.ElevatorConstants.ELEVATOR_POSITION_CONVERSION_FACTOR);
    left.getEncoder()
        .setVelocityConversionFactor(
            Constants.ElevatorConstants.ELEVATOR_VELOCITY_CONVERSION_FACTOR);
    right
        .getEncoder()
        .setVelocityConversionFactor(
            Constants.ElevatorConstants.ELEVATOR_VELOCITY_CONVERSION_FACTOR);


  }

  @Override
  public void updateInputs(ElevatorInputs inputs) {
    inputs.outputVoltage = MathUtil.clamp(left.getOutputCurrent(), -12.0, 12.0);
    inputs.heightInches = Units.metersToInches(left.getEncoder().getPosition());
    inputs.velocityInchesPerSecond = Units.metersToInches(left.getEncoder().getVelocity());
    inputs.tempCelsius = left.getMotorTemperature();
  }

  @Override
  public void setVoltage(double volts) {
    left.setVoltage(volts);
    right.setVoltage(volts);

  }
}
