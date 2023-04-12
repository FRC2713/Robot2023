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

  private CANSparkMax topRoller, bottomRoller;

  public IntakeIOSparks() {
    topRoller = new CANSparkMax(Constants.RobotMap.TOP_INTAKE_ROLLER, MotorType.kBrushless);
    bottomRoller = new CANSparkMax(Constants.RobotMap.BOTTOM_INTAKE_ROLLER, MotorType.kBrushless);
    // topRoller.restoreFactoryDefaults();
    // bottomRoller.restoreFactoryDefaults();

    RedHawkUtil.configureLowTrafficSpark(topRoller);
    RedHawkUtil.configureLowTrafficSpark(bottomRoller);

    // analog sensor voltage, analog sensor velocity, analog sensor position
    topRoller.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 50);
    bottomRoller.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 50);


      topRoller.setInverted(true);
      bottomRoller.setInverted(true);

    topRoller.setSmartCurrentLimit(Constants.IntakeConstants.TOP_CURRENT_LIMIT);
    bottomRoller.setSmartCurrentLimit(Constants.IntakeConstants.BOTTOM_CURRENT_LIMIT);
    topRoller.setIdleMode(IdleMode.kBrake);
    bottomRoller.setIdleMode(IdleMode.kBrake);
    topRoller
        .getEncoder()
        .setPositionConversionFactor(Constants.IntakeConstants.TOP_POSITION_CONVERSION_FACTOR);
    bottomRoller
        .getEncoder()
        .setPositionConversionFactor(Constants.IntakeConstants.BOTTOM_POSITION_CONVERSION_FACTOR);
    topRoller
        .getEncoder()
        .setVelocityConversionFactor(Constants.IntakeConstants.TOP_VELOCITY_CONVERSION_FACTOR);
    bottomRoller
        .getEncoder()
        .setVelocityConversionFactor(Constants.IntakeConstants.BOTTOM_VELOCITY_CONVERSION_FACTOR);
    
        RedHawkUtil.burnSparkFlash(bottomRoller);
        RedHawkUtil.burnSparkFlash(topRoller);
  }

  @Override
  public void updateInputs(IntakeInputs inputs) {
    inputs.topOutputVoltage = MathUtil.clamp(topRoller.getAppliedOutput() * 12, -12.0, 12.0);
    inputs.bottomOutputVoltage = MathUtil.clamp(bottomRoller.getAppliedOutput() * 12, -12.0, 12.0);
    inputs.topIsOn =
        Units.rotationsPerMinuteToRadiansPerSecond(topRoller.getEncoder().getVelocity()) > 0.005;
    inputs.bottomIsOn =
        Units.rotationsPerMinuteToRadiansPerSecond(bottomRoller.getEncoder().getVelocity()) > 0.005;
    inputs.topVelocityRPM = topRoller.getEncoder().getVelocity();
    inputs.bottomVelocityRPM = bottomRoller.getEncoder().getVelocity();
    inputs.topTempCelcius = topRoller.getMotorTemperature();
    inputs.bottomTempCelcius = bottomRoller.getMotorTemperature();
    inputs.topCurrentAmps = topRoller.getOutputCurrent();
    inputs.bottomCurrentAmps = bottomRoller.getOutputCurrent();
    inputs.topPositionRad =
        Units.rotationsPerMinuteToRadiansPerSecond(topRoller.getEncoder().getPosition());
    inputs.bottomPositionRad =
        Units.rotationsPerMinuteToRadiansPerSecond(bottomRoller.getEncoder().getPosition());
    inputs.encoderPositionRight = topRoller.getAnalog(Mode.kAbsolute).getPosition();
    inputs.encoderVelocityRight = topRoller.getAnalog(Mode.kAbsolute).getVelocity();
    inputs.encoderVoltageRight = topRoller.getAnalog(Mode.kAbsolute).getVoltage();

    inputs.encoderPositionLeft = bottomRoller.getAnalog(Mode.kAbsolute).getPosition();
    inputs.encoderVelocityLeft = bottomRoller.getAnalog(Mode.kAbsolute).getVelocity();
    inputs.encoderVoltageLeft = bottomRoller.getAnalog(Mode.kAbsolute).getVoltage();
  }

  @Override
  public void setTopVoltage(double volts) {
    topRoller.setVoltage(volts);
  }

  @Override
  public void setBottomVoltage(double volts) {
    bottomRoller.setVoltage(volts);
  }

  public void setCurrentLimit(int currentLimit) {
    topRoller.setSmartCurrentLimit(currentLimit);
    bottomRoller.setSmartCurrentLimit(currentLimit);
  }
}
