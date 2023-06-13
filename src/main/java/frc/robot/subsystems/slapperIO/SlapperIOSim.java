package frc.robot.subsystems.slapperIO;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

public class SlapperIOSim implements SlapperIO {
  private final SingleJointedArmSim sim =
      new SingleJointedArmSim(
          Constants.SlapperConstants.MOTOR,
          Constants.SlapperConstants.GEARING,
          Constants.SlapperConstants.MOI,
          Constants.SlapperConstants.LENGTH_METRES,
          Units.degreesToRadians(Constants.SlapperConstants.MIN_ANGLE_DEG),
          Units.degreesToRadians(Constants.SlapperConstants.MAX_ANGLE_DEG),
          false);

  @Override
  public void updateInputs(SlapperInputs inputs) {
    if (DriverStation.isDisabled()) {
      sim.setInputVoltage(0.0);
    }
    sim.update(0.02);
    inputs.outputVoltage = MathUtil.clamp(sim.getOutput(0), -12.0, 12.0);
    inputs.isOn = sim.getVelocityRadPerSec() > 0.005;
    inputs.velocityRPM = Units.radiansPerSecondToRotationsPerMinute(sim.getVelocityRadPerSec());

    inputs.tempCelcius = 0.0;
    inputs.currentAmps = sim.getCurrentDrawAmps();
    inputs.positionDeg = Units.radiansToDegrees(sim.getAngleRads());
  }

  @Override
  public void setVoltage(double volts) {
    sim.setInputVoltage(volts);
  }

  public void setCurrentLimit(int currentLimit) {}
}
