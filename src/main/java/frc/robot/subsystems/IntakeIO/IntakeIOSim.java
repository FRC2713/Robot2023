package frc.robot.subsystems.IntakeIO;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class IntakeIOSim implements IntakeIO {
  private final DCMotorSim sim =
      new DCMotorSim(
          DCMotor.getNEO(2), Constants.IntakeConstants.GEARING, Constants.IntakeConstants.MOI);

  @Override
  public void updateInputs(IntakeInputs inputs) {
    if (DriverStation.isDisabled()) {
      sim.setInputVoltage(0.0);
    }
    sim.update(0.02);
    inputs.outputVoltage = MathUtil.clamp(sim.getOutput(0), -12.0, 12.0);
    inputs.isOn = sim.getAngularVelocityRadPerSec() > 0.005;
    inputs.velocityRadsPerSecond = sim.getAngularVelocityRadPerSec();
    inputs.tempCelcius = 0.0;
    inputs.currentAmps = sim.getCurrentDrawAmps();
    inputs.positionRad = sim.getAngularPositionRad();
  }

  @Override
  public void setVoltage(double volts) {
    sim.setInputVoltage(volts);
  }
}
