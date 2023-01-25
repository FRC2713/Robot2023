package frc.robot.subsystems.fourBarIO;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

public class FourBar extends SubsystemBase {

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
    return Math.abs(inputs.angleDegrees - targetRads) < 0.05;
  }

  public double getCurrentRads() {
    return inputs.angleDegrees;
  }

  public void periodic() {
    double effort = controller.calculate(inputs.angleDegrees, targetRads);
    double ffEffort = ff.calculate(inputs.angleDegrees, inputs.velocityDegreesPerSecond);
    effort += ffEffort;
    effort = MathUtil.clamp(effort, -12, 12);

    IO.updateInputs(inputs);
    IO.setVoltage(effort);

    Logger.getInstance().recordOutput("4Bar/Target Rads", targetRads);
    Logger.getInstance().recordOutput("4Bar/Control Effort", effort);
    Logger.getInstance().recordOutput("4Bar/FF Effort", ffEffort);

    Logger.getInstance().processInputs("4Bar", inputs);
  }
  public static class Commands {
    public static Command setToAngle(double angleDeg){
      return new InstantCommand(
              () -> Robot.four.setAngleDeg(angleDeg), Robot.four);
    }
  }
}
