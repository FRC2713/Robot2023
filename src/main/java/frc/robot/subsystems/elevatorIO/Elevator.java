package frc.robot.subsystems.elevatorIO;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

  private final ProfiledPIDController elevatorController;
  private final ElevatorInputsAutoLogged inputs;
  private final ElevatorIO IO;
  private double targetHeight = 0.0;
  private final ElevatorFeedforward feedforward;

  public Elevator(ElevatorIO IO) {
    this.feedforward = new ElevatorFeedforward(0, 5.212, 0);
    this.elevatorController = new ProfiledPIDController(0.6, 0, 0.2, new Constraints(160, 500));
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
    double ffEffort = feedforward.calculate(elevatorController.getSetpoint().velocity);
      effort += ffEffort;
    effort = MathUtil.clamp(effort, -12, 12);

    IO.updateInputs(inputs);
    IO.setVoltage(effort);
    Logger.getInstance().recordOutput("Elevator/Target Height", targetHeight);
    Logger.getInstance().recordOutput("Elevator/Control Effort", effort);
    Logger.getInstance().recordOutput("Elevator/FF Effort", ffEffort);

    Logger.getInstance().processInputs("Elevator", inputs);
  }
}
