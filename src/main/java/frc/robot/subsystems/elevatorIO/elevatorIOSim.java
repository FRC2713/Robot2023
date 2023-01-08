package frc.robot.subsystems.elevatorIO;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;

public class elevatorIOSim implements elevatorIO {

    private ElevatorSim sim = new ElevatorSim(DCMotor.getNEO(1), 10.0, Constants.CONE_MASS_KG, Constants.ELEVATOR_DRUM_RADIUS_METERS, Constants.ELEVATOR_MIN_HEIGHT_METERS, Constants.ELEVATOR_MAX_HEIGHT_METERS, true, VecBuilder.fill(0.0));

    
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
        inputs.velocityMetresPerSecond = sim.getVelocityMetersPerSecond();
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
