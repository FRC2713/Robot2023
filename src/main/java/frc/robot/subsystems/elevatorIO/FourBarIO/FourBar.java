package frc.robot.subsystems.elevatorIO.FourBarIO;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class FourBar extends SubsystemBase {

  private final ProfiledPIDController controller;
  private final FourBarInputsAutoLogged inputs;
  private final FourBarIO IO;
  private double targetRads = 0.0;
  private final ArmFeedforward ff;

  public FourBar(FourBarIO IO) {
    this.ff = new ArmFeedforward(0.0, 0.45, 0.0);
    this.controller = new ProfiledPIDController(50, 0, 0, new Constraints(160, 500));
    this.inputs = new FourBarInputsAutoLogged();
    IO.updateInputs(inputs);
    this.IO = IO;
  }

  //   public void setTargetRads(double targetExtensionInches) {
  //     if (targetExtensionInches > Units.metersToInches(Constants.FourBar.FOUR_BAR_MAX_EXTENSION))
  // {
  //       RedHawkUtil.ErrHandler.getInstance().addError("Target extension too far");
  //       this.targetRads =
  //           MathUtil.clamp(
  //               targetExtensionInches,
  //               0,
  //               Units.metersToInches(Constants.FourBar.FOUR_BAR_MAX_EXTENSION));
  //       return;
  //     }
  //     this.targetRads = targetExtensionInches;
  //   }
  public void setAngleDeg(double targetDegs) {
    this.targetRads = Units.degreesToRadians(-targetDegs);
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
