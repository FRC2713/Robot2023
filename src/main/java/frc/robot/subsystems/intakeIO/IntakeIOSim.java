package frc.robot.subsystems.intakeIO;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class IntakeIOSim implements IntakeIO {
  private final DCMotorSim simTopRollers =
      new DCMotorSim(
          DCMotor.getNeo550(1),
          Constants.IntakeConstants.TOP_GEARING,
          Constants.IntakeConstants.MOI);
  private final DCMotorSim simBottomRollers =
      new DCMotorSim(
          DCMotor.getNeo550(1),
          Constants.IntakeConstants.BOTTOM_GEARING,
          Constants.IntakeConstants.MOI);

  @Override
  public void updateInputs(IntakeInputs inputs) {
    if (DriverStation.isDisabled()) {
      simTopRollers.setInputVoltage(0.0);
      simBottomRollers.setInputVoltage(0.0);
    }
    simTopRollers.update(0.02);
    simBottomRollers.update(0.02);
    inputs.topOutputVoltage = MathUtil.clamp(simTopRollers.getOutput(0), -12.0, 12.0);
    inputs.bottomOutputVoltage = MathUtil.clamp(simBottomRollers.getOutput(0), -12.0, 12.0);
    inputs.topIsOn = simTopRollers.getAngularVelocityRadPerSec() > 0.005;
    inputs.bottomIsOn = simBottomRollers.getAngularVelocityRadPerSec() > 0.005;
    inputs.topVelocityRPM = simTopRollers.getAngularVelocityRPM();

    inputs.bottomVelocityRPM = simBottomRollers.getAngularVelocityRPM();
    inputs.topTempCelcius = 0.0;
    inputs.bottomTempCelcius = 0.0;
    inputs.topCurrentAmps = simTopRollers.getCurrentDrawAmps();
    inputs.bottomCurrentAmps = simBottomRollers.getCurrentDrawAmps();
    inputs.topPositionRad = simTopRollers.getAngularPositionRad();
    inputs.bottomPositionRad = simBottomRollers.getAngularPositionRad();
  }

  @Override
  public void setTopVoltage(double volts) {
    simTopRollers.setInputVoltage(volts);
  }

  @Override
  public void setBottomVoltage(double volts) {
    simBottomRollers.setInputVoltage(volts);
  }

  public void setCurrentLimit(int currentLimit) {}
}
