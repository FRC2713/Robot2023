package frc.robot.subsystems.elevatorIO;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

    private final PIDController elevatorController;
    private final ElevatorIO.ElevatorInputs inputs;
    private final ElevatorIO IO;
    private double targetHeight;

    public Elevator(ElevatorIO IO){
        this.elevatorController = new PIDController(1, 1, 1);
        this.inputs = new ElevatorIO.ElevatorInputs();
        IO.updateInputs(inputs);
        this.IO = IO;
    }
    public void setTargetHeight(double targetHeight){
        this.targetHeight = targetHeight;
        //IO.setVoltage(elevatorController.calculate(inputs.outputVoltage));
    }
    public double getCurrentHeight(){
        return inputs.heightInches;
    }

    public void periodic(){
        IO.updateInputs(inputs);
    }
}
