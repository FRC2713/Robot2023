package frc.robot.subsystems.fourBarIO;

import static frc.robot.Robot.fourBar;

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
    this.ff = Constants.FourBarConstants.FOUR_BAR_GAINS.createArmFeedforward();
    this.controller =
        Constants.FourBarConstants.FOUR_BAR_GAINS.createProfiledPIDController(
            new Constraints(
                Constants.FourBarConstants.MAX_VELOCITY,
                Constants.FourBarConstants.MAX_ACCELERATION));
    this.inputs = new FourBarInputsAutoLogged();
    IO.updateInputs(inputs);
    this.IO = IO;
  }

  public void setAngleDeg(double targetDegs) {
    double rads = Units.degreesToRadians(targetDegs);
    if (rads < Constants.FourBarConstants.EXTENDED_ANGLE_RADIANS
        || rads > Constants.FourBarConstants.RETRACTED_ANGLE_RADIANS) {
      RedHawkUtil.ErrHandler.getInstance().addError("4Bar: Set to degress out of limits range!");
    }
    this.targetDegs = targetDegs;
  }

  public boolean isAtTarget() {
    return Math.abs(inputs.angleDegreesOne - targetDegs) < 0.1;
  }

  public double getCurrentDegs() {
    return inputs.angleDegreesOne;
  }

  public double getCurrentDraw() {
    return inputs.currentDrawOne;
  }

  public void periodic() {
    double effort = controller.calculate(inputs.angleDegreesOne, targetDegs);
    double ffEffort = ff.calculate(Units.radiansToDegrees(inputs.angleDegreesOne), 0);
    effort += ffEffort;
    effort = MathUtil.clamp(effort, -12, 12);

    // if ((inputs.angleDegreesOne
    //     > Units.radiansToDegrees(Constants.FourBarConstants.RETRACTED_ANGLE_RADIANS))) {
    //   effort =
    //       MathUtil.clamp(
    //           controller.calculate(
    //               inputs.angleDegreesOne,
    //               Units.radiansToDegrees(Constants.FourBarConstants.RETRACTED_ANGLE_RADIANS)),
    //           -0.5,
    //           0.5);
    //   RedHawkUtil.ErrHandler.getInstance().addError("4BAR PAST MAX LIMITS!");
    // } else if (inputs.angleDegreesOne
    //     < Units.radiansToDegrees(Constants.FourBarConstants.EXTENDED_ANGLE_RADIANS)) {
    //   effort =
    //       MathUtil.clamp(
    //           controller.calculate(
    //               inputs.angleDegreesOne,
    //               Units.radiansToDegrees(Constants.FourBarConstants.EXTENDED_ANGLE_RADIANS)),
    //           -0.5,
    //           0.5);
    //   RedHawkUtil.ErrHandler.getInstance().addError("4BAR PAST MIN LIMITS!");
    // }

    IO.updateInputs(inputs);
    IO.setVoltage(effort);

    Logger.getInstance().recordOutput("4Bar/Target Degs", targetDegs);
    Logger.getInstance().recordOutput("4Bar/Control Effort", effort);
    Logger.getInstance().recordOutput("4Bar/FF Effort", ffEffort);
    Logger.getInstance().recordOutput("4Bar/atTarget", isAtTarget());

    Logger.getInstance().processInputs("4Bar", inputs);
  }

  public void debugOnlySetVoltage(double volts) {
    // IO.setVoltage(volts);
  }

  public static class Commands {
    public static Command setToAngle(double angleDeg) {
      return new InstantCommand(() -> Robot.fourBar.setAngleDeg(angleDeg), fourBar);
    }

    public static Command setAngleDegAndWait(double targetDegs) {
      return setToAngle(targetDegs).repeatedly().until(() -> Robot.fourBar.isAtTarget());
    }

    public static Command retract() {
      return setToAngle(Units.radiansToDegrees(Constants.FourBarConstants.IDLE_ANGLE_RADIANS));
    }

    public static Command retractFully() {
      return setToAngle(Units.radiansToDegrees(Constants.FourBarConstants.RETRACTED_ANGLE_RADIANS));
    }

    public static Command extend() {
      return setToAngle(Units.radiansToDegrees(Constants.FourBarConstants.EXTENDED_ANGLE_RADIANS));
    }
  }
}
