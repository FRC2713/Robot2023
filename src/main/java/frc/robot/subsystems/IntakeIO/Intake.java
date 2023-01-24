package frc.robot.subsystems.IntakeIO;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Intake {
  private final IntakeIO IO;
  private final IntakeInputsAutoLogged inputs;
  private final PIDController controller;
  private final SimpleMotorFeedforward ff;
  private double targetRadsPerSec = 0.0;

  public Intake(IntakeIO IO) {
    this.inputs = new IntakeInputsAutoLogged();
    this.controller =
        Constants.IntakeConstants.PID_CONTROLLER_FEED_FORWARD.createWpilibController();
    this.ff = Constants.IntakeConstants.PID_CONTROLLER_FEED_FORWARD.createWpilibFeedforward();
    IO.updateInputs(inputs);
    this.IO = IO;
  }

  public boolean isAtTarget() {
    return Math.abs(inputs.velocityRadsPerSecond - targetRadsPerSec) < 0.05;
  }

  public void setVelocityDegPerSec(double targetDegsPerSec) {
    this.targetRadsPerSec = Units.degreesToRadians(targetDegsPerSec);
  }

  public void periodic() {
    double effort = controller.calculate(inputs.velocityRadsPerSecond, targetRadsPerSec);
    double ffEffort = ff.calculate(inputs.velocityRadsPerSecond);
    effort += ffEffort;

    IO.updateInputs(inputs);
    IO.setVoltage(effort);

    Logger.getInstance().recordOutput("Intake/Target RadsPerSec", targetRadsPerSec);
    Logger.getInstance().recordOutput("Intake/Control Effort", effort);
    Logger.getInstance().recordOutput("Intake/FF Effort", ffEffort);
    Logger.getInstance().recordOutput("Intake/Has reached target", isAtTarget());

    Logger.getInstance().processInputs("Intake", inputs);
  }
}
