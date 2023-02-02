package frc.robot.subsystems.fourBarIO;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.RedHawkUtil;
import org.littletonrobotics.junction.Logger;

public class FourBar extends SubsystemBase {

  private final ProfiledPIDController controller;
  private final FourBarInputsAutoLogged inputs;
  private final FourBarIO IO;
  private double targetDegs = 0;
  private final ArmFeedforward ff;

  public FourBar(FourBarIO IO) {
    this.ff = Constants.FourBarConstants.PID_CONTROLLER_FEED_FORWARD.createArmFeedforward();
    this.controller =
        Constants.FourBarConstants.PID_CONTROLLER_FEED_FORWARD.createProfiledPIDController(
            new Constraints(
                Constants.FourBarConstants.MAX_VELOCITY,
                Constants.FourBarConstants.MAX_ACCELERATION));
    this.inputs = new FourBarInputsAutoLogged();
    IO.updateInputs(inputs);
    this.IO = IO;
  }

  public void setAngleDeg(double targetDegs) {
    double rads = Units.degreesToRadians(targetDegs);
    if (rads < Constants.FourBarConstants.MIN_ANGLE_RADIANS
        || rads > Constants.FourBarConstants.MAX_ANGLE_RADIANS) {
      RedHawkUtil.ErrHandler.getInstance().addError("4Bar: Set to degress out of limits range!");
    }
    this.targetDegs = targetDegs;
  }

  public Command cmdSetAngleDeg(double targetDegs) {
    return new InstantCommand(() -> Robot.four.setAngleDeg(targetDegs));
  }

  public Command cmdSetAngleDegAndWait(double targetDegs) {
    return cmdSetAngleDeg(targetDegs).repeatedly().until(() -> isAtTarget());
  }

  public Command cmdRetract() {
    return cmdSetAngleDeg(Units.radiansToDegrees(Constants.FourBarConstants.MAX_ANGLE_RADIANS));
  }

  public Command cmdExtend() {
    return cmdSetAngleDeg(Units.radiansToDegrees(Constants.FourBarConstants.MIN_ANGLE_RADIANS));
  }

  public boolean isAtTarget() {
    return Math.abs(inputs.angleDegrees - targetDegs) < 0.1;
  }

  public double getCurrentDegs() {
    return inputs.angleDegrees;
  }

  public void periodic() {
    double effort = controller.calculate(inputs.angleDegrees, targetDegs);
    double ffEffort = ff.calculate(inputs.angleDegrees, inputs.velocityDegreesPerSecond);
    effort += ffEffort;
    effort = MathUtil.clamp(effort, -12, 12);

    if ((inputs.angleDegrees
        > Units.radiansToDegrees(Constants.FourBarConstants.MAX_ANGLE_RADIANS))) {
      effort =
          MathUtil.clamp(
              controller.calculate(
                  inputs.angleDegrees,
                  Units.radiansToDegrees(Constants.FourBarConstants.MAX_ANGLE_RADIANS)),
              -0.5,
              0.5);
      RedHawkUtil.ErrHandler.getInstance().addError("4BAR PAST MAX LIMITS!");
    } else if (inputs.angleDegrees
        < Units.radiansToDegrees(Constants.FourBarConstants.MIN_ANGLE_RADIANS)) {
      effort =
          MathUtil.clamp(
              controller.calculate(
                  inputs.angleDegrees,
                  Units.radiansToDegrees(Constants.FourBarConstants.MIN_ANGLE_RADIANS)),
              -0.5,
              0.5);
      RedHawkUtil.ErrHandler.getInstance().addError("4BAR PAST MIN LIMITS!");
    }

    IO.updateInputs(inputs);
    IO.setVoltage(effort);

    Logger.getInstance().recordOutput("4Bar/Target Degs", targetDegs);
    Logger.getInstance().recordOutput("4Bar/Control Effort", effort);
    Logger.getInstance().recordOutput("4Bar/FF Effort", ffEffort);
    Logger.getInstance().recordOutput("4Bar/atTarget", isAtTarget());

    Logger.getInstance().processInputs("4Bar", inputs);
  }

  public static class Commands {
    public static Command setToAngle(double angleDeg) {
      return new InstantCommand(() -> Robot.four.setAngleDeg(angleDeg), Robot.four);
    }
  }
}
