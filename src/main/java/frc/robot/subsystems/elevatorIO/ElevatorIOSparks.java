package frc.robot.subsystems.elevatorIO;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import frc.robot.util.RedHawkUtil;
import java.util.HashMap;

public class ElevatorIOSparks implements ElevatorIO {
  private CANSparkMax left, right;

  public ElevatorIOSparks() {
    left = new CANSparkMax(Constants.RobotMap.ELEVATOR_LEFT_CANID, MotorType.kBrushless);
    right = new CANSparkMax(Constants.RobotMap.ELEVATOR_RIGHT_CANID, MotorType.kBrushless);
    left.restoreFactoryDefaults();
    right.restoreFactoryDefaults();

    RedHawkUtil.configureCANSparkMAXStatusFrames(
        new HashMap<>() {
          {
            put(PeriodicFrame.kStatus0, 60);
            put(PeriodicFrame.kStatus1, 20);
            put(PeriodicFrame.kStatus2, 20);
            put(PeriodicFrame.kStatus3, 65535);
            put(PeriodicFrame.kStatus4, 65535);
            put(PeriodicFrame.kStatus5, 65535);
            put(PeriodicFrame.kStatus6, 65535);
          }
        },
        left,
        right);

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

    left.burnFlash();
    right.burnFlash();
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

  public void setPIDFF() {
    SparkMaxPIDController leftController = left.getPIDController();
    SparkMaxPIDController rightController = right.getPIDController();

    leftController.setP(Constants.ElevatorConstants.ELEVATOR_GAINS.kP.get());
    leftController.setI(Constants.ElevatorConstants.ELEVATOR_GAINS.kI.get());
    leftController.setD(Constants.ElevatorConstants.ELEVATOR_GAINS.kD.get());
    leftController.setFF(Constants.ElevatorConstants.ELEVATOR_GAINS.kG.get());
    rightController.setP(Constants.ElevatorConstants.ELEVATOR_GAINS.kP.get());
    rightController.setI(Constants.ElevatorConstants.ELEVATOR_GAINS.kI.get());
    rightController.setD(Constants.ElevatorConstants.ELEVATOR_GAINS.kD.get());
    rightController.setFF(Constants.ElevatorConstants.ELEVATOR_GAINS.kG.get());
  }

  @Override
  public void updatePID(double heightInchesRight, double setpoint, double ffVolts) {
    if (shouldApplyFF()) {
      left.getPIDController()
          .setReference(setpoint, CANSparkMax.ControlType.kPosition, 0, ffVolts);
      right
          .getPIDController()
          .setReference(setpoint, CANSparkMax.ControlType.kPosition, 0, ffVolts);
    } else {
      left.getPIDController().setReference(setpoint, CANSparkMax.ControlType.kPosition, 0);
      right.getPIDController().setReference(setpoint, CANSparkMax.ControlType.kPosition, 0);
    }
  }
  
}
