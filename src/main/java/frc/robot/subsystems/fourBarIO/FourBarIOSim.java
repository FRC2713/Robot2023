package frc.robot.subsystems.fourBarIO;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

public class FourBarIOSim implements FourBarIO {
  private final SingleJointedArmSim sim =
      new SingleJointedArmSim(
          DCMotor.getNEO(1),
          Constants.FourBarConstants.GEARING,
          SingleJointedArmSim.estimateMOI(
              Constants.FourBarConstants.LENGTH_METRES, Constants.FourBarConstants.MASS_KG),
          Constants.FourBarConstants.LENGTH_METRES,
          Constants.FourBarConstants.MIN_ANGLE_RADIANS,
          Constants.FourBarConstants.MAX_ANGLE_RADIANS,
          true);

  @Override
  public void updateInputs(FourBarInputs inputs) {
    if (DriverStation.isDisabled()) {
      sim.setInputVoltage(0.0);
    }
    sim.update(0.02);
    inputs.outputVoltage = MathUtil.clamp(sim.getOutput(0), -12.0, 12.0);
    inputs.angleDegrees = Units.radiansToDegrees(sim.getAngleRads());
    inputs.velocityDegreesPerSecond = Units.radiansToDegrees(sim.getVelocityRadPerSec());
    inputs.tempCelcius = 0.0;
    inputs.currentDrawAmps = sim.getCurrentDrawAmps();
  }

  @Override
  public void setVoltage(double volts) {
    sim.setInputVoltage(volts);
  }
}
