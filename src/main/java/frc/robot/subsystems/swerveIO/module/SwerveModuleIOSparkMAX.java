package frc.robot.subsystems.swerveIO.module;

import static frc.robot.util.RedHawkUtil.cOk;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import frc.robot.util.OffsetAbsoluteAnalogEncoder;
import frc.robot.util.PIDFFGains;
import frc.robot.util.RedHawkUtil;
import java.util.HashMap;

public class SwerveModuleIOSparkMAX implements SwerveModuleIO {

  OffsetAbsoluteAnalogEncoder azimuthEncoder;
  CANSparkMax driver;
  CANSparkMax azimuth;
  private final ModuleInfo information;
  PIDFFGains aziGains;
  PIDFFGains driveGains;
  SparkMaxPIDController azController;
  SparkMaxPIDController driveController;
  private double aziSetpointDeg;
  private double driveSetpointMPS;
  private SimpleMotorFeedforward aziFF;
  private SimpleMotorFeedforward driveFF;

  private RelativeEncoder getDriveEncoder() {
    return driver.getEncoder();
  }

  private RelativeEncoder getAziEncoder() {
    return azimuth.getEncoder();
  }

  private OffsetAbsoluteAnalogEncoder getAziAbsoluteEncoder() {
    return azimuthEncoder;
  }

  public void applyVoltageForCharacterization(double voltage) {
    driver.setVoltage(voltage);
  }

  public SwerveModuleIOSparkMAX(ModuleInfo information) {
    this.information = information;
    azimuthEncoder =
        new OffsetAbsoluteAnalogEncoder(
            this.information.getAziEncoderCANId(), this.information.getOffset());
    driver = new CANSparkMax(this.information.getDriveCANId(), MotorType.kBrushless);
    azimuth = new CANSparkMax(this.information.getAziCANId(), MotorType.kBrushless);

    driver.restoreFactoryDefaults();
    azimuth.restoreFactoryDefaults();

    driver.setCANTimeout(Constants.CAN_TIMEOUT_MS);
    azimuth.setCANTimeout(Constants.CAN_TIMEOUT_MS);

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
        driver,
        azimuth);

    driver.setSmartCurrentLimit(Constants.DriveConstants.DRIVE_CURRENT_LIMIT);
    azimuth.setSmartCurrentLimit(Constants.DriveConstants.AZI_CURRENT_LIMIT);

    for (int i = 0; i < 30; i++) {
      azimuth.setInverted(true);
      driver.setInverted(true);
    }

    cOk(driver.setIdleMode(IdleMode.kCoast));
    cOk(azimuth.setIdleMode(IdleMode.kBrake));

    cOk(getDriveEncoder().setPositionConversionFactor(Constants.DriveConstants.DIST_PER_PULSE));
    cOk(
        getDriveEncoder()
            .setVelocityConversionFactor((Constants.DriveConstants.DIST_PER_PULSE / 60)));

    cOk(getAziEncoder().setPositionConversionFactor(7.0 / 150.0 * 360.0));
    cOk(getAziEncoder().setVelocityConversionFactor(7.0 / 150.0 * 360.0));

    for (int i = 0; i < 30; i++) {
      // seed();
    }

    driver.setCANTimeout(0);
    azimuth.setCANTimeout(0);

    driver.burnFlash();
    azimuth.burnFlash();

    azController = azimuth.getPIDController();
    driveController = driver.getPIDController();

    this.aziGains = information.getAzimuthGains();
    this.aziFF = aziGains.createWpilibFeedforward();
    RedHawkUtil.errorHandleSparkMAX(azimuth.getPIDController().setP(aziGains.kP.get()));
    RedHawkUtil.errorHandleSparkMAX(azimuth.getPIDController().setI(aziGains.kI.get()));
    RedHawkUtil.errorHandleSparkMAX(azimuth.getPIDController().setD(aziGains.kD.get()));
    azimuth.getPIDController().setPositionPIDWrappingEnabled(true);

    this.driveGains = information.getDriveGains();
    this.driveFF = driveGains.createWpilibFeedforward();
    RedHawkUtil.errorHandleSparkMAX(driver.getPIDController().setP(driveGains.kP.get()));
    RedHawkUtil.errorHandleSparkMAX(driver.getPIDController().setI(driveGains.kI.get()));
    RedHawkUtil.errorHandleSparkMAX(driver.getPIDController().setD(driveGains.kD.get()));
    driver.getPIDController().setPositionPIDWrappingEnabled(true);
  }

  public void seed() {
    cOk(getAziEncoder().setPosition(getAziAbsoluteEncoder().getAdjustedRotation2d().getDegrees()));
  }

  @Override
  public void updateInputs(SwerveModuleInputs inputs) {
    // inputs.aziAbsoluteEncoderRawVoltsReal = getAziEncoder().getPosition();
    // inputs.aziAbsoluteEncoderAdjVoltsReal = getAziEncoder().getPosition();
    // inputs.aziAbsoluteEncoderAdjAngleDegReal = getAziEncoder().getPosition();

    inputs.aziAbsoluteEncoderRawVolts = azimuthEncoder.getUnadjustedVoltage();
    inputs.aziAbsoluteEncoderAdjVolts = azimuthEncoder.getAdjustedVoltage();
    inputs.aziAbsoluteEncoderAdjAngleDeg = azimuthEncoder.getAdjustedRotation2d().getDegrees();

    inputs.aziOutputVolts = azimuth.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.aziTempCelcius = azimuth.getMotorTemperature();
    inputs.aziCurrentDrawAmps = azimuth.getOutputCurrent();
    inputs.aziEncoderPositionDeg = getAziEncoder().getPosition();
    inputs.aziEncoderVelocityDegPerSecond = getAziEncoder().getVelocity();
    inputs.aziEncoderSimplifiedPositionDeg =
        OffsetAbsoluteAnalogEncoder.simplifyRotation2d(
                Rotation2d.fromDegrees(getAziEncoder().getPosition()))
            .getDegrees();

    inputs.driveEncoderPositionMetres = getDriveEncoder().getPosition();
    inputs.driveEncoderVelocityMetresPerSecond = getDriveEncoder().getVelocity();
    inputs.driveOutputVolts = driver.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.driveCurrentDrawAmps = driver.getOutputCurrent();
    inputs.driveTempCelcius = driver.getMotorTemperature();

    azController.setReference(
        aziSetpointDeg,
        ControlType.kPosition,
        0,
        aziFF.calculate(inputs.aziEncoderVelocityDegPerSecond));
    driveController.setReference(
        driveSetpointMPS,
        ControlType.kVelocity,
        0,
        driveFF.calculate(inputs.driveEncoderVelocityMetresPerSecond));
  }

  @Override
  public void setAzimuthVoltage(double aziVolts) {
    azimuth.setVoltage(aziVolts);
  }

  @Override
  public void setDriveVoltage(double driveVolts) {
    driver.setVoltage(driveVolts);
  }

  @Override
  public void setDrivePIDSetpoint(double driveSetpointMPS) {
    this.driveSetpointMPS = driveSetpointMPS;
    azController.setReference(driveSetpointMPS, ControlType.kVelocity);
  }

  @Override
  public void setAziPIDSetpoint(double aziSetpointDeg) {
    this.aziSetpointDeg = aziSetpointDeg;
    driveController.setReference(aziSetpointDeg, ControlType.kPosition);
  }
}
