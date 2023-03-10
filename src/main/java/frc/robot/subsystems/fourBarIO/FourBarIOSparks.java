package frc.robot.subsystems.fourBarIO;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;
import frc.robot.util.RedHawkUtil;

public class FourBarIOSparks implements FourBarIO {
  private CANSparkMax fourBarOne, fourBarTwo;
  private SparkMaxAbsoluteEncoder absoluteEncoder;

  public FourBarIOSparks() {
    fourBarOne = new CANSparkMax(Constants.RobotMap.FOURBAR_ONE_CANID, MotorType.kBrushless);
    // fourBarTwo = new CANSparkMax(Constants.RobotMap.FOURBAR_TWO_CANID, MotorType.kBrushless);
    fourBarOne.restoreFactoryDefaults();
    // fourBarTwo.restoreFactoryDefaults();

    RedHawkUtil.configureHighTrafficSpark(fourBarOne);
    // RedHawkUtil.configureHighTrafficSpark(fourBarTwo);

    fourBarOne.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 200);
    fourBarOne.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 200);

    fourBarOne.setIdleMode(IdleMode.kBrake);

    fourBarOne.setInverted(true); // subject to change
    // fourBarTwo.setInverted(true); // subject to change
    fourBarOne.setSmartCurrentLimit(Constants.FourBarConstants.FOUR_BAR_CURRENT_LIMIT);
    // fourBarTwo.setSmartCurrentLimit(Constants.FourBarConstants.FOUR_BAR_CURRENT_LIMIT);
    fourBarOne
        .getEncoder()
        .setPositionConversionFactor(Constants.FourBarConstants.FOUR_BAR_ANGLE_CONVERSION);
    // fourBarTwo
    //     .getEncoder()
    //     .setPositionConversionFactor(Constants.FourBarConstants.FOUR_BAR_ANGLE_CONVERSION);
    fourBarOne
        .getEncoder()
        .setVelocityConversionFactor(
            Constants.FourBarConstants.FOUR_BAR_VELOCITY_CONVERSION_FACTOR);
    // fourBarTwo
    //     .getEncoder()
    //     .setVelocityConversionFactor(
    //         Constants.FourBarConstants.FOUR_BAR_VELOCITY_CONVERSION_FACTOR);

    if (Math.abs(fourBarOne.getEncoder().getPosition()) < 0.01) {
      fourBarOne.getEncoder().setPosition(Constants.FourBarConstants.RETRACTED_ANGLE_DEGREES);
    }

    absoluteEncoder = fourBarOne.getAbsoluteEncoder(Type.kDutyCycle);
    absoluteEncoder.setPositionConversionFactor(1 / 2.5 * 360);
    absoluteEncoder.setVelocityConversionFactor(1 / 2.5 * 360);
    absoluteEncoder.setInverted(true);

    absoluteEncoder.setZeroOffset(10);
    // absoluteEncoder.setZeroOffset(
    //     -Units.radiansToDegrees(FourBarConstants.RETRACTED_ANGLE_RADIANS) + 20);
  }

  @Override
  public void updateInputs(FourBarInputs inputs) {
    inputs.outputVoltage = MathUtil.clamp(fourBarOne.getOutputCurrent(), -12, 12);

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
}
