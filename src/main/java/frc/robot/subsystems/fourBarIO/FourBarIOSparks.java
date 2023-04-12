package frc.robot.subsystems.fourBarIO;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import frc.robot.util.RedHawkUtil;
import org.littletonrobotics.junction.Logger;

public class FourBarIOSparks implements FourBarIO {
  private CANSparkMax fourBarOne, fourBarTwo;
  private SparkMaxAbsoluteEncoder absoluteEncoder;

  private static double offset = 173.95;

  public FourBarIOSparks() {
    fourBarOne = new CANSparkMax(Constants.RobotMap.FOURBAR_ONE_CANID, MotorType.kBrushless);
    // fourBarTwo = new CANSparkMax(Constants.RobotMap.FOURBAR_TWO_CANID, MotorType.kBrushless);
    // fourBarOne.restoreFactoryDefaults();
    // fourBarTwo.restoreFactoryDefaults();

    RedHawkUtil.configureHighTrafficSpark(fourBarOne);
    // RedHawkUtil.configureHighTrafficSpark(fourBarTwo);
    fourBarOne.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 20);
    fourBarOne.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 20);
    fourBarOne.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 10);
    fourBarOne.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 10);

    fourBarOne.setIdleMode(IdleMode.kBrake);

    fourBarOne.setInverted(true);
    fourBarOne.setSmartCurrentLimit(Constants.FourBarConstants.FOUR_BAR_BASE_CURRENT);
    fourBarOne
        .getEncoder()
        .setPositionConversionFactor(Constants.FourBarConstants.FOUR_BAR_ANGLE_CONVERSION);

    fourBarOne
        .getEncoder()
        .setVelocityConversionFactor(
            Constants.FourBarConstants.FOUR_BAR_VELOCITY_CONVERSION_FACTOR);

    if (Math.abs(fourBarOne.getEncoder().getPosition()) < 0.01) {
      fourBarOne.getEncoder().setPosition(Constants.FourBarConstants.RETRACTED_ANGLE_DEGREES);
    }

    absoluteEncoder = fourBarOne.getAbsoluteEncoder(Type.kDutyCycle);
    absoluteEncoder.setPositionConversionFactor(360);
    absoluteEncoder.setVelocityConversionFactor(360);
    absoluteEncoder.setInverted(false);

    absoluteEncoder.setZeroOffset(0);

    RedHawkUtil.burnSparkFlash(fourBarOne);
  }

  @Override
  public void updateInputs(FourBarInputs inputs) {
    inputs.outputVoltage = (fourBarOne.getAppliedOutput() * RobotController.getBatteryVoltage());

    inputs.angleDegreesOne = fourBarOne.getEncoder().getPosition();
    // inputs.angleDegreesTwo = fourBarTwo.getEncoder().getPosition();
    inputs.angleDegreesRange =
        // fourBarOne.getEncoder().getPosition(); - fourBarTwo.getEncoder().getPosition();

        inputs.velocityDegreesPerSecondOne = fourBarOne.getEncoder().getVelocity();
    // inputs.velocityDegreesPerSecondTwo = fourBarTwo.getEncoder().getVelocity();
    inputs.velocityDegreesPerSecondRange =
        fourBarOne.getEncoder().getVelocity(); // - fourBarTwo.getEncoder().getVelocity();

    inputs.tempCelciusOne = fourBarOne.getMotorTemperature();
    // inputs.tempCelciusTwo = fourBarTwo.getMotorTemperature();

    inputs.currentDrawOne = fourBarOne.getOutputCurrent();
    // inputs.currentDrawTwo = fourBarTwo.getOutputCurrent();

    inputs.absoluteEncoderVolts = absoluteEncoder.getPosition();
    inputs.absoluteEncoderAdjustedAngle = inputs.absoluteEncoderVolts - (offset);
    // inputs.limSwitch =
    //     fourBarOne.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen).isPressed();
    Logger.getInstance().recordOutput("4Bar/isInverted", fourBarOne.getInverted());
  }

  @Override
  public void setVoltage(double volts) {
    fourBarOne.setVoltage(volts);
    // fourBarTwo.setVoltage(volts);
  }

  @Override
  public void setPosition(double angleDeg) {
    fourBarOne.getEncoder().setPosition(angleDeg);
  }

  public void reseed(double absoluteEncoderVolts) {
    var trueAngle = absoluteEncoderVolts - offset;
    fourBarOne.getEncoder().setPosition(trueAngle);
  }

  @Override
  public void setCurrentLimit(int currentLimit) {
    fourBarOne.setSmartCurrentLimit(currentLimit);
  }
}
