package frc.robot.subsystems.slapperIO;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxAnalogSensor.Mode;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.util.RedHawkUtil;

public class SlapperIOSparks implements SlapperIO {

  private CANSparkMax slapperMotor;

  public SlapperIOSparks() {
    slapperMotor = new CANSparkMax(Constants.RobotMap.CONE_SLAPPER_MOTOR, MotorType.kBrushless);
    slapperMotor.restoreFactoryDefaults();

    RedHawkUtil.configureLowTrafficSpark(slapperMotor);

    // analog sensor voltage, analog sensor velocity, analog sensor position
    slapperMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 50);

    slapperMotor.setInverted(true);

    slapperMotor.setSmartCurrentLimit(Constants.SlapperConstants.CURRENT_LIMIT);
    slapperMotor.setIdleMode(IdleMode.kBrake);
    slapperMotor
        .getEncoder()
        .setPositionConversionFactor(Constants.SlapperConstants.POSITION_CONVERSION_FACTOR);
    slapperMotor
        .getEncoder()
        .setVelocityConversionFactor(Constants.SlapperConstants.VELOCITY_CONVERSION_FACTOR);

    slapperMotor.getEncoder().setPosition(Constants.SlapperConstants.MAX_ANGLE_DEG);
  }

  @Override
  public void updateInputs(SlapperInputs inputs) {
    inputs.outputVoltage = MathUtil.clamp(slapperMotor.getAppliedOutput() * 12, -12.0, 12.0);
    inputs.isOn =
        Units.rotationsPerMinuteToRadiansPerSecond(slapperMotor.getEncoder().getVelocity()) > 0.005;
    inputs.velocityRPM = slapperMotor.getEncoder().getVelocity();
    inputs.tempCelcius = slapperMotor.getMotorTemperature();
    inputs.currentAmps = slapperMotor.getOutputCurrent();
    inputs.positionDeg = slapperMotor.getEncoder().getPosition();
    inputs.encoderPosition = slapperMotor.getAnalog(Mode.kAbsolute).getPosition();
    inputs.encoderVelocity = slapperMotor.getAnalog(Mode.kAbsolute).getVelocity();
    inputs.encoderVoltage = slapperMotor.getAnalog(Mode.kAbsolute).getVoltage();
  }

  @Override
  public void setVoltage(double volts) {
    slapperMotor.setVoltage(volts);
  }

  public void setCurrentLimit(int currentLimit) {
    slapperMotor.setSmartCurrentLimit(currentLimit);
  }
}
