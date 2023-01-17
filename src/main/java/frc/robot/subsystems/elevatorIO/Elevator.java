package frc.robot.subsystems.elevatorIO;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.RedHawkUtil;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

  private final ProfiledPIDController elevatorController;
  private final ElevatorInputsAutoLogged inputs;
  private final ElevatorIO IO;
  private double targetHeight = 0.0;
  private final ElevatorFeedforward feedforward;

  public Elevator(ElevatorIO IO) {
    this.feedforward = new ElevatorFeedforward(0, 5.212, 0);
    this.elevatorController = new ProfiledPIDController(0.15, 0, 0.1, new Constraints(160, 500));
    SmartDashboard.putData("Elevator PID", elevatorController);
    this.inputs = new ElevatorInputsAutoLogged();
    IO.updateInputs(inputs);
    this.IO = IO;
  }

  public void setTargetHeight(double targetHeightInches) {
    if (targetHeightInches > Units.metersToInches(Constants.Elevator.ELEVATOR_MAX_HEIGHT_METERS)) {
      RedHawkUtil.ErrHandler.getInstance().addError("Target height too high");
      this.targetHeight =
          MathUtil.clamp(
              targetHeightInches,
              0,
              Units.metersToInches(Constants.Elevator.ELEVATOR_MAX_HEIGHT_METERS));
      return;
    }
    this.targetHeight = targetHeightInches;
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
