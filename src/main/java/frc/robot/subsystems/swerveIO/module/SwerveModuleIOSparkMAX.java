package frc.robot.subsystems.swerveIO.module;

import static frc.robot.util.RedHawkUtil.cOk;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import frc.robot.util.OffsetAbsoluteAnalogEncoder;
import frc.robot.util.RedHawkUtil;

public class SwerveModuleIOSparkMAX implements SwerveModuleIO {

  OffsetAbsoluteAnalogEncoder azimuthEncoder;
  CANSparkMax driver;
  CANSparkMax azimuth;
  private final ModuleInfo information;

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

    // driver.restoreFactoryDefaults();
    // azimuth.restoreFactoryDefaults();

    driver.setCANTimeout(Constants.CAN_TIMEOUT_MS);
    azimuth.setCANTimeout(Constants.CAN_TIMEOUT_MS);

    RedHawkUtil.configureHighTrafficSpark(azimuth);
    RedHawkUtil.configureHighTrafficSpark(driver);

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
  }

  public void seed() {
    cOk(getAziEncoder().setPosition(getAziAbsoluteEncoder().getAdjustedRotation2d().getDegrees()));
  }

  @Override
  public void updateInputs(SwerveModuleInputs inputs) {
    // inputs.aziAbsoluteEncoderRawVoltsReal = getAziEncoder().getPosition();
    // inputs.aziAbsoluteEncoderAdjVoltsReal = getAziEncoder().getPosition();
    // inputs.aziAbsoluteEncoderAdjAngleDegReal = getAziEncoder().getPosition();

    inputs.aziAbsoluteEncoderRawVoltsReal = azimuthEncoder.getUnadjustedVoltage();
    inputs.aziAbsoluteEncoderAdjVoltsReal = azimuthEncoder.getAdjustedVoltage();
    inputs.aziAbsoluteEncoderAdjAngleDegReal =
        azimuthEncoder.getAdjustedRotation2d().getDegrees();

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
  }

  @Override
  public void setAzimuthVoltage(double aziVolts) {
    azimuth.setVoltage(aziVolts);
  }

  @Override
  public void setDriveVoltage(double driveVolts) {
    driver.setVoltage(driveVolts);
  }
}
