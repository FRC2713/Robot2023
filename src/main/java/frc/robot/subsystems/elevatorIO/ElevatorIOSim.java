package frc.robot.subsystems.elevatorIO;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;

public class ElevatorIOSim implements ElevatorIO {

  private final AngledElevatorSim sim =
      new AngledElevatorSim(
          DCMotor.getNEO(2),
          Constants.ElevatorConstants.ELEVATOR_GEAR_RATIO,
          Constants.ElevatorConstants.CARRIAGE_MASS_KG,
          Constants.ElevatorConstants.ELEVATOR_DRUM_RADIUS_METERS,
          Constants.ElevatorConstants.ELEVATOR_MIN_HEIGHT_METERS,
          Constants.ElevatorConstants.ELEVATOR_MAX_HEIGHT_METERS,
          true,
          null,
          Rotation2d.fromDegrees(Constants.ElevatorConstants.ELEVATOR_ANGLE_DEGREES));

  public void resetEncoders() {}
  /**
   * Automatically updates given {@code ElevatorInputs} instance based on simulation
   *
   * @param inputs a ElevatorInputs instance
   */
  @Override
  public void updateInputs(ElevatorInputs inputs) {
    if (DriverStation.isDisabled()) {
      sim.setInputVoltage(0.0);
    }
    sim.update(0.02);
    inputs.outputVoltageLeft = MathUtil.clamp(sim.getOutput(0), -12.0, 12.0);
    inputs.heightInchesLeft = Units.metersToInches(sim.getPositionMeters());
    inputs.velocityInchesPerSecondLeft = Units.metersToInches(sim.getVelocityMetersPerSecond());
    inputs.tempCelsiusLeft = 0.0;
    inputs.currentDrawAmpsLeft = sim.getCurrentDrawAmps();
    inputs.outputVoltageRight = MathUtil.clamp(sim.getOutput(0), -12.0, 12.0);
    inputs.heightInchesRight = Units.metersToInches(sim.getPositionMeters());
    inputs.velocityInchesPerSecondRight = Units.metersToInches(sim.getVelocityMetersPerSecond());
    inputs.tempCelsiusRight = 0.0;
    inputs.currentDrawAmpsRight = sim.getCurrentDrawAmps();
  }

  public boolean shouldApplyFF() {
    return true;
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
