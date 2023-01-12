package frc.robot.subsystems.elevatorIO;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class ElevatorIOSim implements ElevatorIO {
  private final Mechanism2d mech;
  private final MechanismRoot2d root;
  private final MechanismLigament2d m_elevator;

  public ElevatorIOSim() {
    // the main mechanism object

    mech = new Mechanism2d(2, 50);

    // the mechanism root node

    root = mech.getRoot("climber", 2, 0);
    m_elevator = root.append(new MechanismLigament2d("elevator", 0, 90));
    SmartDashboard.putData("Mech2d", mech);
  }

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
    if (DriverStation.isDisabled()) {
      return;
    }
    sim.update(0.02);
    inputs.outputVoltage = MathUtil.clamp(sim.getOutput(0), -12.0, 12.0);
    inputs.heightInches = Units.metersToInches(sim.getPositionMeters());
    inputs.velocityInchesPerSecond = Units.metersToInches(sim.getVelocityMetersPerSecond());
    inputs.tempCelcius = 0.0;

    m_elevator.setLength(0 + inputs.heightInches);
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
