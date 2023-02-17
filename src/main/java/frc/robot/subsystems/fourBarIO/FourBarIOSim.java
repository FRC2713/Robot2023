package frc.robot.subsystems.fourBarIO;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

public class FourBarIOSim implements FourBarIO {
  private static final SingleJointedArmSim sim =
      new SingleJointedArmSim(
          DCMotor.getNEO(1),
          Constants.FourBarConstants.GEARING,
          SingleJointedArmSim.estimateMOI(
              Constants.FourBarConstants.LENGTH_METRES, Constants.FourBarConstants.MASS_KG),
          Constants.FourBarConstants.LENGTH_METRES,
          Constants.FourBarConstants.MAX_ANGLE_RADIANS,
          Constants.FourBarConstants.RETRACTED_ANGLE_RADIANS,
          true);

  static {
    sim.setState(VecBuilder.fill(Constants.FourBarConstants.RETRACTED_ANGLE_RADIANS, 0));
  }

  @Override
  public void updateInputs(FourBarInputs inputs) {
    if (DriverStation.isDisabled()) {
      sim.setInputVoltage(0.0);
    }
    sim.update(0.02);

    inputs.outputVoltage = MathUtil.clamp(sim.getOutput(0), -12.0, 12.0);

    inputs.angleDegreesOne = Units.radiansToDegrees(sim.getAngleRads());
    inputs.angleDegreesTwo = Units.radiansToDegrees(sim.getAngleRads());
    inputs.angleDegreesRange = 0.0;

    inputs.velocityDegreesPerSecondOne = Units.radiansToDegrees(sim.getVelocityRadPerSec());
    inputs.velocityDegreesPerSecondTwo = Units.radiansToDegrees(sim.getVelocityRadPerSec());
    inputs.velocityDegreesPerSecondRange = 0.0;

    inputs.tempCelciusOne = 0.0;
    inputs.tempCelciusTwo = 0.0;

    inputs.currentDrawOne = sim.getCurrentDrawAmps();
    inputs.currentDrawTwo = sim.getCurrentDrawAmps();
  }

  @Override
  public void setVoltage(double volts) {
    sim.setInputVoltage(volts);
  }
}
