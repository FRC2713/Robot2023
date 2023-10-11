package frc.robot.subsystems.elevatorIO;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class ElevatorIOSim implements ElevatorIO {
  private final ProfiledPIDController elevatorController;

  public ElevatorIOSim() {
    this.elevatorController =
        Constants.ElevatorConstants.ELEVATOR_GAINS.createProfiledPIDController(
            new Constraints(100, 200));
  }

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
    inputs.currentDrawAmpsLeft = sim.getCurrentDrawAmps() / 2.0;
    inputs.outputVoltageRight = MathUtil.clamp(sim.getOutput(0), -12.0, 12.0);
    inputs.heightInchesRight = Units.metersToInches(sim.getPositionMeters());
    inputs.velocityInchesPerSecondRight = Units.metersToInches(sim.getVelocityMetersPerSecond());
    inputs.tempCelsiusRight = 0.0;
    inputs.currentDrawAmpsRight = sim.getCurrentDrawAmps() / 2.0;
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
  public void setPIDFF() {
    elevatorController.setP(Constants.ElevatorConstants.ELEVATOR_GAINS.kP.get());
    elevatorController.setI(Constants.ElevatorConstants.ELEVATOR_GAINS.kI.get());
    elevatorController.setD(Constants.ElevatorConstants.ELEVATOR_GAINS.kD.get());
    elevatorController.setConstraints(new Constraints(100, 200));
  }

  @Override
  public void updatePID(double heightInchesRight, double setpoint, double ffVolts) {

    double targetVoltage = elevatorController.calculate(heightInchesRight, setpoint);

    if (shouldApplyFF()) {
      targetVoltage += ffVolts;
    }

    targetVoltage = MathUtil.clamp(targetVoltage, -12, 12);
    sim.setInputVoltage(targetVoltage);
    Logger.getInstance().recordOutput("Elevator/Control Voltage", targetVoltage);
  }
}
