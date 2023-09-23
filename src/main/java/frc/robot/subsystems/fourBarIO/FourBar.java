package frc.robot.subsystems.fourBarIO;

import static frc.robot.Robot.fourBar;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.FourBarConstants;
import frc.robot.Robot;
import frc.robot.Robot.GamePieceMode;
import frc.robot.util.LoggableMotor;
import frc.robot.util.RedHawkUtil;
import frc.robot.util.SuperstructureConfig;
import org.littletonrobotics.junction.Logger;

public class FourBar extends SubsystemBase {

  public enum FourBarMode {
    CLOSED_LOOP,
    OPEN_LOOP,
    HOMING
  }

  private FourBarMode mode;

  private final ProfiledPIDController voltageController;
  private final PIDController currentController;
  public final FourBarInputsAutoLogged inputs;
  private final FourBarIO IO;
  private double targetDegs = Constants.FourBarConstants.RETRACTED_ANGLE_DEGREES;
  private final ArmFeedforward ff;
  private Timer timer = new Timer();
  private double absOffset = 0;
  private LoggableMotor motor = new LoggableMotor("FourBar", DCMotor.getNEO(1));

  public FourBar(FourBarIO IO) {
    this.ff = Constants.FourBarConstants.FOUR_BAR_VOLTAGE_GAINS.createArmFeedforward();
    this.voltageController =
        Constants.FourBarConstants.FOUR_BAR_VOLTAGE_GAINS.createProfiledPIDController(
            new Constraints(FourBarConstants.MAX_VELOCITY, FourBarConstants.MAX_ACCELERATION));
    this.currentController =
        Constants.FourBarConstants.FOUR_BAR_CURRENT_GAINS.createWpilibController();
    this.inputs = new FourBarInputsAutoLogged();
    this.IO = IO;
    this.IO.updateInputs(inputs);
    // setMode(FourBarMode.HOMING);
    setMode(FourBarMode.CLOSED_LOOP);
    // reseed();
  }

  public void setAngleDeg(double targetDegs) {
    if (targetDegs < Constants.FourBarConstants.EXTENDED_ANGLE_DEGREES
        || targetDegs > Constants.FourBarConstants.RETRACTED_ANGLE_DEGREES) {
      RedHawkUtil.ErrHandler.getInstance().addError("4Bar: Set to degress out of limits range!");
    }
    // voltageController.reset(Units.degreesToRadians(inputs.angleDegreesOne),
    // Units.degreesToRadians(inputs.velocityDegreesPerSecondOne));
    this.targetDegs = targetDegs;
    mode = FourBarMode.CLOSED_LOOP;
  }

  public boolean isAtTarget() {
    return Math.abs(inputs.absoluteEncoderAdjustedAngle - targetDegs) < 2;
  }

  public double getCurrentDegs() {
    return inputs.absoluteEncoderAdjustedAngle;
  }

  public void reseed() {
    absOffset = inputs.absoluteEncoderAdjustedAngle - inputs.angleDegreesOne;
    // IO.reseed(inputs.absoluteEncoderVolts);
    // targetDegs += 0.01;
    reset();
  }

  public double getCurrentDraw() {
    return inputs.currentDrawOne;
  }

  public void setPosition(double angleDegs) {
    IO.setPosition(angleDegs);
  }

  public double computeOffsetAngle() {
    return absOffset + inputs.angleDegreesOne;
  }

  public void reset() {
    // voltageController.reset(
    //     Units.degreesToRadians(computeOffsetAngle()),
    //     Units.degreesToRadians(inputs.velocityDegreesPerSecondOne));
  }

  public void setMode(FourBarMode newMode) {
    if (this.mode != FourBarMode.HOMING && newMode == FourBarMode.HOMING) {
      timer.reset();
      timer.start();
      IO.setPosition(0);
    } else if (this.mode == FourBarMode.HOMING && newMode == FourBarMode.CLOSED_LOOP) {
      reset();
    }

    this.mode = newMode;
  }

  public boolean isHomed(boolean useEncoders) {
    // both methods work in sim
    return (useEncoders
        ? (timer.get() > 1 && inputs.velocityDegreesPerSecondOne < 1)
        : inputs.limSwitch);
  }

  public void periodic() {
    IO.updateInputs(inputs);
    Logger.getInstance().processInputs("4Bar", inputs);
    motor.log(inputs.currentDrawOne, inputs.outputVoltage);

    double voltage = 0;
    switch (mode) {
      case CLOSED_LOOP:
        {
          boolean shouldReset =
              Math.abs(inputs.absoluteEncoderAdjustedAngle - inputs.angleDegreesOne) > 3;
          if (shouldReset) {
            // reseed();
          }
          double effort =
              voltageController.calculate(
                  // absoluteEncoderAdjustedAngle, angleDegreesOne
                  Units.degreesToRadians(inputs.absoluteEncoderAdjustedAngle),
                  Units.degreesToRadians(targetDegs));

          var goal = voltageController.getGoal();
          var setpoint = voltageController.getSetpoint();

          Logger.getInstance()
              .recordOutput("4Bar/Goal/Position", Units.radiansToDegrees(goal.position));
          Logger.getInstance()
              .recordOutput("4Bar/Goal/Velocity", Units.radiansToDegrees(goal.velocity));
          Logger.getInstance()
              .recordOutput("4Bar/Setpoint/Position", Units.radiansToDegrees(setpoint.position));
          Logger.getInstance()
              .recordOutput("4Bar/Setpoint/Velocity", Units.radiansToDegrees(setpoint.velocity));

          Logger.getInstance().recordOutput("4Bar/Should Reseed", shouldReset);

          Logger.getInstance().recordOutput("4Bar/Control Effort", effort);

          double ffEffort = ff.calculate(setpoint.position, setpoint.velocity);
          Logger.getInstance().recordOutput("4Bar/FF Effort", ffEffort);

          effort += ffEffort;
          effort = MathUtil.clamp(effort, -12, 12);
          Logger.getInstance().recordOutput("4Bar/Total Effort", effort);

          voltage = effort;

          double current = FourBarConstants.FOUR_BAR_MAX_CURRENT;
          // MathUtil.clamp(
          //     Math.abs(currentController.calculate(inputs.angleDegreesOne, targetDegs))
          //         + FourBarConstants.FOUR_BAR_BASE_CURRENT,
          //     0,
          //     FourBarConstants.FOUR_BAR_MAX_CURRENT);
          IO.setCurrentLimit((int) current);
          Logger.getInstance().recordOutput("4Bar/Current Limit", current);
        }
        break;
      case HOMING:
        {
          voltage = Constants.FourBarConstants.HOMING_VOLTAGE;
          IO.setCurrentLimit(Constants.FourBarConstants.FOUR_BAR_MAX_CURRENT);
          if (isHomed(true)) {
            IO.setPosition(Constants.FourBarConstants.RETRACTED_ANGLE_DEGREES);
            targetDegs = Constants.FourBarConstants.IDLE_ANGLE_DEGREES;
            setMode(FourBarMode.CLOSED_LOOP);
            voltage = 0;
          }
        }
        break;
      case OPEN_LOOP:
        voltage = -1 * Robot.operator.getLeftX();
        break;
    }

    IO.setVoltage(voltage);

    Logger.getInstance().recordOutput("4Bar/Computed Offset Angle", computeOffsetAngle());
    Logger.getInstance().recordOutput("4Bar/Offset Angle", absOffset);

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

    public static Command setDrawVolts(int volts) {
      return new InstantCommand(() -> Robot.fourBar.setVoltage(volts));
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
