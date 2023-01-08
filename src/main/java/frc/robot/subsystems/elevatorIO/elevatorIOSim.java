package frc.robot.subsystems.elevatorIO;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;

public class elevatorIOSim implements elevatorIO{

    ElevatorSim sim = new ElevatorSim(DCMotor.getNEO(1), 10.0, Constants.CONE_MASS_KG, Constants.ELEVATOR_DRUM_RADIUS_METERS, Constants.ELEVATOR_MIN_HEIGHT_METERS, Constants.ELEVATOR_MAX_HEIGHT_METERS, true, VecBuilder.fill(0.0));

    @Override
    public void updateInputs(ElevatorInputs inputs) {
        sim.update(0.02);
    }

    @Override
    public void setVoltage(double volts) {

    }
}
