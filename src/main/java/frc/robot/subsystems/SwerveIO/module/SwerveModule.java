package frc.robot.subsystems.SwerveIO.module;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.PIDFFController;
import org.littletonrobotics.junction.Logger;

public class SwerveModule extends SubsystemBase {

  SwerveModuleIO io;
  SwerveModuleName name;
  public final SwerveModuleInputsAutoLogged inputs = new SwerveModuleInputsAutoLogged();

  private final PIDFFController driveController;
  private final PIDFFController azimuthController;

  SwerveModuleState state;
  public final ModuleInfo information;

  /**
   * Creates a new SwerveModule object.
   *
   * @param swerveModuleIO The IO layer. Change this to change which motor controllers you're using
   *     (SwerveModuleIOSim vs SwerveModuleIOSparkMAX)
   * @param name The name of the swerve module (how it shows up in logging tools)
   */
  public SwerveModule(SwerveModuleIO swerveModuleIO, ModuleInfo information) {
    this.information = information;
    this.driveController = new PIDFFController(this.information.getDriveGains());
    this.azimuthController = new PIDFFController(this.information.getAzimuthGains());

    io = swerveModuleIO;
    this.name = this.information.getName();
    io.updateInputs(inputs);

    state = new SwerveModuleState(0, Rotation2d.fromDegrees(inputs.aziEncoderPositionDeg));
    azimuthController.enableContinuousInput(-180, 180);
  }

  /**
   * Returns the current objective state of the swerve drive.
   *
   * @return The desired SwerveModuleState object.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        inputs.driveEncoderVelocityMetresPerSecond,
        Rotation2d.fromDegrees(inputs.aziEncoderPositionDeg));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        inputs.driveEncoderPositionMetres, Rotation2d.fromDegrees(inputs.aziEncoderPositionDeg));
  }
  // Only used to characterize the drive
  public double getVoltageAppliedForCharacterization() {
    return inputs.driveOutputVolts;
  }

  // Only used to characterize the drive
  public void applyVoltageForCharacterization(double voltage) {
    io.setDriveVoltage(voltage);
  }

  /**
   * Optimizes the given SwerveModuleState and make it the setpoint of the swerve module.
   *
   * @param desiredState The new setpoint of the swerve module.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    state =
        SwerveModuleState.optimize(
            desiredState, Rotation2d.fromDegrees(inputs.aziEncoderPositionDeg));
  }

  /**
   * Recalculates the voltage outputs of the drive and azimuth voltages and sets them. Should run on
   * every code loop, so put it in periodic() for best results.
   */
  public void update() {
    final double driveOutput =
        driveController.calculate(
            inputs.driveEncoderVelocityMetresPerSecond, state.speedMetersPerSecond);
    final double turnOutput =
        azimuthController.calculate(inputs.aziEncoderPositionDeg, state.angle.getDegrees());

    applyVoltageForCharacterization(driveOutput);
    io.setAzimuthVoltage(turnOutput);
  }

  @Override
  public void periodic() {
    update();

    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Swerve/" + name, inputs);
    Logger.getInstance()
        .recordOutput(
            "Swerve/" + name + "/Azimuth Error",
            state.angle.getDegrees() - inputs.aziEncoderPositionDeg);
    Logger.getInstance()
        .recordOutput(
            "Swerve/" + name + "/Drive Error",
            state.speedMetersPerSecond - inputs.driveEncoderVelocityMetresPerSecond);

    Logger.getInstance()
        .recordOutput(
            "Swerve/" + name + "/Azimuth Encoder Delta",
            inputs.aziEncoderPositionDeg - inputs.aziAbsoluteEncoderAdjAngleDeg);
  }
}
