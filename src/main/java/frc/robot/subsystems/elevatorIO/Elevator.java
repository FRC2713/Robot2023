package frc.robot.subsystems.elevatorIO;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

  private final PIDController elevatorController;
  private final ElevatorInputsAutoLogged inputs;
  private final ElevatorIO IO;
  private double targetHeight = 0.0;

  public Elevator(ElevatorIO IO) {
    this.elevatorController = new PIDController(2, 0, 0);
    this.inputs = new ElevatorInputsAutoLogged();
    IO.updateInputs(inputs);
    this.IO = IO;
  }

  public void setTargetHeight(double targetHeight) {
    this.targetHeight = targetHeight;
  }

  public double getCurrentHeight() {
    return inputs.heightInches;
  }

  public void periodic() {
    double effort = elevatorController.calculate(inputs.heightInches, targetHeight);
    IO.updateInputs(inputs);
    IO.setVoltage(effort);
    Logger.getInstance().recordOutput("Elevator/Target Height", targetHeight);
    Logger.getInstance().recordOutput("Elevator/Control Effort", effort);
    Logger.getInstance().processInputs("Elevator", inputs);
  }
}
