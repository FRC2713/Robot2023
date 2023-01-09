package frc.robot.subsystems.elevatorIO;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;

public class ElevatorIOSim implements ElevatorIO {

  private ElevatorSim sim =
      new ElevatorSim(
          DCMotor.getNEO(2),
          1.0,
          Constants.Elevator.CARRIAGE_MASS_KG,
          Constants.Elevator.ELEVATOR_DRUM_RADIUS_METERS,
          Constants.Elevator.ELEVATOR_MIN_HEIGHT_METERS,
          Constants.Elevator.ELEVATOR_MAX_HEIGHT_METERS,
          true);

  /**
   * Automatically updates given {@code ElevatorInputs} instance based on simulation
   *
   * @param inputs a ElevatorInputs instance
   */
  @Override
  public void updateInputs(ElevatorInputs inputs) {
    sim.update(0.02);
    inputs.outputVoltage = MathUtil.clamp(sim.getOutput(0), -12.0, 12.0);
    inputs.heightInches = Units.metersToInches(sim.getPositionMeters());
    inputs.velocityInchesPerSecond = Units.metersToInches(sim.getVelocityMetersPerSecond());
    inputs.tempCelcius = 0.0;
  }

  /**
   * Sets the simulation's input voltage to the given {@code volts}
   *
   * @param volts the voltage to set it to
   */
  @Override
  public void setVoltage(double volts) {
    sim.setInputVoltage(volts);
  }
}
