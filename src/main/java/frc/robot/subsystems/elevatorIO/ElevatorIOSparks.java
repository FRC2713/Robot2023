package frc.robot.subsystems.elevatorIO;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import frc.robot.util.RedHawkUtil;

public class ElevatorIOSparks implements ElevatorIO {
  private CANSparkMax left, right;

  public ElevatorIOSparks() {
    left = new CANSparkMax(Constants.RobotMap.ELEVATOR_LEFT_CANID, MotorType.kBrushless);
    right = new CANSparkMax(Constants.RobotMap.ELEVATOR_RIGHT_CANID, MotorType.kBrushless);

    // left.restoreFactoryDefaults();
    // right.restoreFactoryDefaults();

    RedHawkUtil.configureHighTrafficSpark(left);
    RedHawkUtil.configureHighTrafficSpark(right);

    left.setIdleMode(CANSparkMax.IdleMode.kBrake);
    right.setIdleMode(CANSparkMax.IdleMode.kBrake);

    for (int i = 0; i < 30; i++) {
      left.setInverted(true);
      right.setInverted(false);
    }

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

    RedHawkUtil.burnSparkFlash(left);
    RedHawkUtil.burnSparkFlash(right);
  }

  public void resetEncoders() {
    left.getEncoder().setPosition(0);
    right.getEncoder().setPosition(0);
  }

  public boolean shouldApplyFF() {
    return right.getEncoder().getPosition() > 28.25;
  }

  @Override
  public void updateInputs(ElevatorInputs inputs) {
    inputs.outputVoltageLeft =
        MathUtil.clamp(left.getAppliedOutput() * RobotController.getBatteryVoltage(), -12.0, 12.0);
    inputs.heightInchesLeft = left.getEncoder().getPosition();
    inputs.velocityInchesPerSecondLeft = left.getEncoder().getVelocity();
    inputs.tempCelsiusLeft = left.getMotorTemperature();
    inputs.currentDrawAmpsLeft = left.getOutputCurrent();

    inputs.outputVoltageRight =
        MathUtil.clamp(right.getAppliedOutput() * RobotController.getBatteryVoltage(), -12.0, 12.0);
    inputs.heightInchesRight = right.getEncoder().getPosition();
    inputs.velocityInchesPerSecondRight = right.getEncoder().getVelocity();
    inputs.tempCelsiusRight = right.getMotorTemperature();
    inputs.currentDrawAmpsRight = right.getOutputCurrent();
  }

  @Override
  public void setVoltage(double volts) {
    left.setVoltage(volts);
    right.setVoltage(volts);
  }
}
