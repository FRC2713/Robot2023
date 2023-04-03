package frc.robot.subsystems.fourBarIO;

import static frc.robot.Robot.fourBar;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.FourBarConstants;
import frc.robot.Robot;
import frc.robot.Robot.GamePieceMode;
import frc.robot.util.RedHawkUtil;
import frc.robot.util.SuperstructureConfig;
import org.littletonrobotics.junction.Logger;

public class FourBar extends SubsystemBase {

  public enum FourBarMode {
    CLOSED_LOOP,
    OPEN_LOOP,
    HOMING
  }

  private FourBarMode mode = FourBarMode.CLOSED_LOOP;

  private final PIDController voltageController;
  private final PIDController currentController;
  public final FourBarInputsAutoLogged inputs;
  private final FourBarIO IO;
  private double targetDegs = Constants.FourBarConstants.RETRACTED_ANGLE_DEGREES;
  private final ArmFeedforward ff;

  public FourBar(FourBarIO IO) {
    this.ff = Constants.FourBarConstants.FOUR_BAR_VOLTAGE_GAINS.createArmFeedforward();
    this.voltageController =
        Constants.FourBarConstants.FOUR_BAR_VOLTAGE_GAINS.createWpilibController();
    this.currentController =
        Constants.FourBarConstants.FOUR_BAR_CURRENT_GAINS.createWpilibController();
    this.inputs = new FourBarInputsAutoLogged();
    IO.updateInputs(inputs);
    this.IO = IO;
  }

  public void setAngleDeg(double targetDegs) {
    if (targetDegs < Constants.FourBarConstants.EXTENDED_ANGLE_DEGREES
        || targetDegs > Constants.FourBarConstants.RETRACTED_ANGLE_DEGREES) {
      RedHawkUtil.ErrHandler.getInstance().addError("4Bar: Set to degress out of limits range!");
    }

    this.targetDegs = targetDegs;
  }

  public boolean isAtTarget() {
    return Math.abs(inputs.angleDegreesOne - targetDegs) < 2;
  }

  public double getCurrentDegs() {
    return inputs.angleDegreesOne;
  }

  public void reseed() {
    IO.reseed(inputs.absoluteEncoderVolts);
  }

  public double getCurrentDraw() {
    return inputs.currentDrawOne;
  }

  public void setPosition(double angleDegs) {
    IO.setPosition(angleDegs);
  }

  public void setMode(FourBarMode newMode) {
    if (this.mode != FourBarMode.HOMING && newMode == FourBarMode.HOMING) {
      IO.setPosition(0);
    }

    this.mode = newMode;
  }

  public boolean isHomed(boolean useEncoders) {
    // both methods work in sim
    return useEncoders
        ? (inputs.angleDegreesOne > 1 && inputs.velocityDegreesPerSecondOne < 1)
        : inputs.limSwitch;
  }

  public void periodic() {
    IO.updateInputs(inputs);
    Logger.getInstance().processInputs("4Bar", inputs);

    double voltage = 0;
    switch (mode) {
      case CLOSED_LOOP:
        {
          double effort = voltageController.calculate(inputs.angleDegreesOne, targetDegs);

          double current =
              MathUtil.clamp(
                  Math.abs(currentController.calculate(inputs.angleDegreesOne, targetDegs))
                      + FourBarConstants.FOUR_BAR_BASE_CURRENT,
                  0,
                  FourBarConstants.FOUR_BAR_MAX_CURRENT);
          IO.setCurrentLimit((int) current);

          Logger.getInstance().recordOutput("4Bar/Control Effort", effort);
          Logger.getInstance().recordOutput("4Bar/Current Limit", current);
          double ffEffort =
              Math.cos(Units.degreesToRadians(inputs.angleDegreesOne))
                  * FourBarConstants.FOUR_BAR_VOLTAGE_GAINS.kG.get();
          effort += ffEffort;
          effort = MathUtil.clamp(effort, -12, 12);
          voltage = effort;
          Logger.getInstance().recordOutput("4Bar/Total Effort", effort);
          Logger.getInstance().recordOutput("4Bar/FF Effort", ffEffort);
        }
        break;
      case HOMING:
        {
          voltage = Constants.FourBarConstants.HOMING_VOLTAGE;
          IO.setCurrentLimit(Constants.FourBarConstants.FOUR_BAR_MAX_CURRENT);
          if (isHomed(false)) {
            IO.setPosition(Constants.FourBarConstants.RETRACTED_ANGLE_DEGREES);
            setMode(FourBarMode.CLOSED_LOOP);
            voltage = 0;
          }
        }
        break;
      case OPEN_LOOP:
        break;
    }

    IO.setVoltage(voltage);

    Logger.getInstance().recordOutput("4Bar/Output", voltage);
    Logger.getInstance().recordOutput("4Bar/Mode", mode.name());

    Logger.getInstance().recordOutput("4Bar/Target Degs", targetDegs);
    Logger.getInstance().recordOutput("4Bar/atTarget", isAtTarget());
  }

  public static class Commands {

    public static ConditionalCommand conditionalFourbarHigh() {
      return new ConditionalCommand(
          FourBar.Commands.setAngleDegAndWait(
              Constants.SuperstructureConstants.SCORE_CONE_HIGH.getFourBarPosition()),
          FourBar.Commands.setAngleDegAndWait(
              Constants.SuperstructureConstants.SCORE_CUBE_HIGH.getFourBarPosition()),
          () -> Robot.gamePieceMode == GamePieceMode.CONE);
    }

    public static ConditionalCommand conditionalFourbarMid() {
      return new ConditionalCommand(
          FourBar.Commands.setAngleDegAndWait(
              Constants.SuperstructureConstants.SCORE_CONE_MID.getFourBarPosition()),
          FourBar.Commands.setAngleDegAndWait(
              Constants.SuperstructureConstants.SCORE_CUBE_MID.getFourBarPosition()),
          () -> Robot.gamePieceMode == GamePieceMode.CONE);
    }

    public static ConditionalCommand conditionalFourbarLow() {
      return new ConditionalCommand(
          FourBar.Commands.setAngleDegAndWait(
              Constants.SuperstructureConstants.SCORE_CONE_LOW.getFourBarPosition()),
          FourBar.Commands.setAngleDegAndWait(
              Constants.SuperstructureConstants.SCORE_CUBE_LOW.getFourBarPosition()),
          () -> Robot.gamePieceMode == GamePieceMode.CONE);
    }

    public static Command setToAngle(double angleDeg) {
      return new InstantCommand(() -> Robot.fourBar.setAngleDeg(angleDeg), fourBar);
    }

    public static Command setToAngle(SuperstructureConfig config) {
      return setToAngle(config.getFourBarPosition());
    }

    public static Command setAngleDegAndWait(double targetDegs) {
      return setToAngle(targetDegs).repeatedly().until(() -> Robot.fourBar.isAtTarget());
    }

    public static Command setAngleDegAndWait(SuperstructureConfig config) {
      return setAngleDegAndWait(config.getFourBarPosition());
    }

    public static Command retract() {
      return setToAngle(Constants.FourBarConstants.IDLE_ANGLE_DEGREES);
    }

    public static Command retractFully() {
      return setToAngle(Constants.FourBarConstants.RETRACTED_ANGLE_DEGREES);
    }

    public static Command extend() {
      return setToAngle(Constants.FourBarConstants.EXTENDED_ANGLE_DEGREES);
    }

    public static Command reset() {
      return new SequentialCommandGroup(
          new InstantCommand(
              () -> {
                Robot.fourBar.setMode(FourBarMode.HOMING);
              })
          /*
          new RunCommand((() -> Robot.fourBar.setVoltage(3)))
              .until(() -> Robot.fourBar.getLimitSwitch()),
          new InstantCommand(
              () -> {
                Robot.fourBar.setPosition(Constants.FourBarConstants.RETRACTED_ANGLE_DEGREES);
                Robot.fourBar.setVoltage(0);
                Robot.fourBar.setMode(FourBarMode.CLOSED_LOOP);
              })
          */
          );
    }
  }

  public void setVoltage(int i) {
    IO.setVoltage(i);
  }

  public boolean getLimitSwitch() {
    return inputs.limSwitch;
  }
}
