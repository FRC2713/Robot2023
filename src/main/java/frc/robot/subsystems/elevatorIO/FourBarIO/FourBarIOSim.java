package frc.robot.subsystems.elevatorIO.FourBarIO;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.subsystems.elevatorIO.FourBarIO.FourBarIO.FourBarInputs;

public class FourBarIOSim implements FourBarIO {

  private static final double kArmEncoderDistPerPulse = 2.0 * Math.PI / 4096;

  // Simulation classes help us simulate what's going on, including gravity.
  private static final double m_armReduction = 600;
  private static final double m_armMass = 0.5; // Kilograms
  private static final double m_armLength = Units.inchesToMeters(11.315);
  // This arm sim represents an arm that can travel from -75 degrees (rotated down front)
  // to 255 degrees (rotated down in the back).
  private final SingleJointedArmSim sim =
      new SingleJointedArmSim(
          DCMotor.getNEO(2),
          m_armReduction,
          SingleJointedArmSim.estimateMOI(m_armLength, m_armMass),
          m_armLength,
          Units.degreesToRadians(63.75),
          Units.degreesToRadians(203.75),
          m_armMass,
          true,
          VecBuilder.fill(kArmEncoderDistPerPulse) // Add noise with a std-dev of 1 tick
          );

  @Override
  public void updateInputs(FourBarInputs inputs) {
    if (DriverStation.isDisabled()) {
      sim.setInputVoltage(0.0);
    }
    sim.update(0.02);
    inputs.outputVoltage = MathUtil.clamp(sim.getOutput(0), -12.0, 12.0);
    inputs.angleRads = sim.getAngleRads();
    inputs.velocityRadsPerSecond = sim.getVelocityRadPerSec();
    inputs.tempCelcius = 0.0;
  }

  @Override
  public void setVoltage(double volts) {
    sim.setInputVoltage(volts);
  }
}
