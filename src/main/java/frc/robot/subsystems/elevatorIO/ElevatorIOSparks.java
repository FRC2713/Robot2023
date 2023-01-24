package frc.robot.subsystems.elevatorIO;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;

public class ElevatorIOSparks implements ElevatorIO {
  private CANSparkMax left, right;

  public ElevatorIOSparks() {
    left = new CANSparkMax(0, MotorType.kBrushless);
    right = new CANSparkMax(1, MotorType.kBrushless);
    left.setSmartCurrentLimit(50);
    right.setSmartCurrentLimit(50);
    left.getEncoder()
        .setPositionConversionFactor(Constants.Elevator.ELEVATOR_POSITION_CONVERSION_FACTOR);
    right
        .getEncoder()
        .setPositionConversionFactor(Constants.Elevator.ELEVATOR_POSITION_CONVERSION_FACTOR);
    left.getEncoder()
        .setVelocityConversionFactor(Constants.Elevator.ELEVATOR_VELOCITY_CONVERSION_FACTOR);
    right
        .getEncoder()
        .setVelocityConversionFactor(Constants.Elevator.ELEVATOR_VELOCITY_CONVERSION_FACTOR);
  }

  @Override
  public void updateInputs(ElevatorInputs inputs) {
    if (DriverStation.isDisabled()) {
      return;
    }
    inputs.outputVoltage = MathUtil.clamp(left.getOutputCurrent(), -12.0, 12.0);
    inputs.heightInches = Units.metersToInches(left.getEncoder().getPosition());
    inputs.velocityInchesPerSecond = Units.metersToInches(left.getEncoder().getVelocity());
    inputs.tempCelsius = left.getMotorTemperature();
  }

  @Override
  public void setVoltage(double volts) {
    left.setVoltage(1);
    right.setVoltage(1);
  }
}
