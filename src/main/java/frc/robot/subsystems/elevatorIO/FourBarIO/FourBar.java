package frc.robot.subsystems.elevatorIO.FourBarIO;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class FourBar {

  private final ProfiledPIDController controller;
  private final FourBarInputsAutoLogged inputs;
  private final FourBarIO IO;
  private double targetRads = Units.degreesToRadians(0);
  private final ArmFeedforward ff;

  public FourBar(FourBarIO IO) {
    this.ff = Constants.FourBarConstants.FEED_FORWARD;
    this.controller = Constants.FourBarConstants.PID_CONTROLLER;
    this.inputs = new FourBarInputsAutoLogged();
    IO.updateInputs(inputs);
    this.IO = IO;
  }

  public void setAngleDeg(double targetDegs) {
    this.targetRads = Units.degreesToRadians(targetDegs);
  }

  public boolean isAtTarget() {
    return Math.abs(inputs.angleRads - targetRads) < 0.05;
  }

  public double getCurrentRads() {
    return inputs.angleRads;
  }

  public void periodic() {
    double effort = controller.calculate(inputs.angleRads, targetRads);
    double ffEffort = ff.calculate(inputs.angleRads, inputs.velocityRadsPerSecond);
    effort += ffEffort;
    effort = MathUtil.clamp(effort, -12, 12);

    IO.updateInputs(inputs);
    IO.setVoltage(effort);

    Logger.getInstance().recordOutput("4Bar/Target Rads", targetRads);
    Logger.getInstance().recordOutput("4Bar/Control Effort", effort);
    Logger.getInstance().recordOutput("4Bar/FF Effort", ffEffort);

    Logger.getInstance().processInputs("4Bar", inputs);
  }
}
