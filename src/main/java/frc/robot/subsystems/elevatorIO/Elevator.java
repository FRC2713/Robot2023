package frc.robot.subsystems.elevatorIO;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Robot.GamePieceMode;
import frc.robot.util.RedHawkUtil;
import frc.robot.util.SuperstructureConfig;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

  private final ProfiledPIDController elevatorController;
  private final ElevatorInputsAutoLogged inputs;
  private final ElevatorIO IO;
  private double targetHeight = 0.0;
  private final ElevatorFeedforward feedforward;

  public Elevator(ElevatorIO IO) {
    this.feedforward = Constants.ElevatorConstants.ELEVATOR_GAINS.createElevatorFeedforward();
    this.elevatorController =
        Constants.ElevatorConstants.ELEVATOR_GAINS.createProfiledPIDController(
            new Constraints(100, 200));
    SmartDashboard.putData("Elevator PID", elevatorController);
    this.inputs = new ElevatorInputsAutoLogged();
    IO.updateInputs(inputs);
    this.IO = IO;
  }

  public void setTargetHeight(double targetHeightInches) {
    if (targetHeightInches > (Constants.ElevatorConstants.ELEVATOR_MAX_HEIGHT_INCHES)) {
      RedHawkUtil.ErrHandler.getInstance().addError("Target height too high");
      this.targetHeight =
          MathUtil.clamp(
              targetHeightInches, 0, (Constants.ElevatorConstants.ELEVATOR_MAX_HEIGHT_INCHES));
      return;
    }
    this.targetHeight = targetHeightInches;
  }

  public double getCurrentHeight() {
    return inputs.heightInchesRight;
  }

  public double getCurrentDraw() {
    return inputs.currentDrawAmpsLeft + inputs.currentDrawAmpsRight;
  }

  public double getTargetHeight() {
    return targetHeight;
  }

  public boolean atTargetHeight() {
    return Math.abs(getCurrentHeight() - targetHeight) < 1;
  }

  public void periodic() {
    double effortLeft =
        elevatorController.calculate(
            // (inputs.heightInchesLeft + inputs.heightInchesRight) / 2, targetHeight); left encoder
            // returns 0 for a cycle sometimes
            inputs.heightInchesRight, targetHeight);
    // double ffEffort = feedforward.calculate(0);
    effortLeft += 0.625;
    effortLeft = MathUtil.clamp(effortLeft, -12, 12);

    Logger.getInstance()
        .recordOutput("Elevator/Setpoint Velocity", elevatorController.getSetpoint().velocity);
    Logger.getInstance()
        .recordOutput("Elevator/Setpoint Position", elevatorController.getSetpoint().position);

    IO.updateInputs(inputs);
    IO.setVoltage(effortLeft);
    Logger.getInstance().recordOutput("Elevator/Target Height", targetHeight);
    Logger.getInstance().recordOutput("Elevator/Control Effort", effortLeft);
    // Logger.getInstance().recordOutput("Elevator/FF Effort", ffEffort);
    Logger.getInstance().recordOutput("Elevator/isAtTarget", atTargetHeight());
    // Logger.getInstance().recordOutput("Elevator/shouldReset", shouldResetEncoders());

    Logger.getInstance().recordOutput("Elevator/heightInchesLeft", inputs.heightInchesLeft);
    Logger.getInstance().recordOutput("Elevator/heightInchesRight", inputs.heightInchesRight);

    Logger.getInstance().processInputs("Elevator", inputs);
    // if (inputs.heightInchesLeft
    //         <= Units.metersToInches(Constants.ElevatorConstants.ELEVATOR_MIN_HEIGHT_METERS)
    //     && inputs.velocityInchesPerSecondLeft < 0) {
    //   IO.setVoltage(0);
    //   return;
    // }
    // if (inputs.heightInchesLeft
    //         >= Units.metersToInches(Constants.ElevatorConstants.ELEVATOR_MAX_HEIGHT_METERS)
    //     && inputs.velocityInchesPerSecondLeft > 0) {
    //   IO.setVoltage(0);
    //   return;
    // }
    // if (inputs.heightInchesRight
    //         <= Units.metersToInches(Constants.ElevatorConstants.ELEVATOR_MIN_HEIGHT_METERS)
    //     && inputs.velocityInchesPerSecondRight < 0) {
    //   IO.setVoltage(0);
    //   return;
    // }
    // if (inputs.heightInchesRight
    //         >= Units.metersToInches(Constants.ElevatorConstants.ELEVATOR_MAX_HEIGHT_METERS)
    //     && inputs.velocityInchesPerSecondRight > 0) {
    //   IO.setVoltage(0);
    //   return;
    // }
  }

  public static class Commands {

    public static Command setToHeightAndWait(double targetHeightInches) {
      return setToHeight(targetHeightInches)
          .repeatedly()
          .until(() -> Robot.elevator.atTargetHeight());
    }

    public static Command setToHeight(double height) {
      return new InstantCommand(() -> Robot.elevator.setTargetHeight(height), Robot.elevator);
    }

    public static Command setToHeight(SuperstructureConfig config) {
      return setToHeight(config.getElevatorPosition());
    }

    public static Command setToHeightAndWait(SuperstructureConfig config) {
      return setToHeightAndWait(config.getElevatorPosition());
    }

    public static Command elevatorCurrentHeight() {
      return new InstantCommand(
          () -> Robot.elevator.setTargetHeight(Robot.elevator.getCurrentHeight()), Robot.elevator);
    }

    public static ConditionalCommand conditionalElevatorHigh() {
      return new ConditionalCommand(
          Elevator.Commands.setToHeightAndWait(
              Constants.SuperstructureConstants.SCORE_CONE_HIGH.getElevatorPosition()),
          Elevator.Commands.setToHeightAndWait(
              Constants.SuperstructureConstants.SCORE_CUBE_HIGH.getElevatorPosition()),
          () -> Robot.gamePieceMode == GamePieceMode.CONE);
    }

    public static ConditionalCommand conditionalElevatorMid() {
      return new ConditionalCommand(
          Elevator.Commands.setToHeightAndWait(
              Constants.SuperstructureConstants.SCORE_CONE_MID.getElevatorPosition()),
          Elevator.Commands.setToHeightAndWait(
              Constants.SuperstructureConstants.SCORE_CUBE_MID.getElevatorPosition()),
          () -> Robot.gamePieceMode == GamePieceMode.CONE);
    }

    public static ConditionalCommand conditionalElevatorLow() {
      return new ConditionalCommand(
          Elevator.Commands.setToHeightAndWait(
              Constants.SuperstructureConstants.SCORE_CONE_LOW.getElevatorPosition()),
          Elevator.Commands.setToHeightAndWait(
              Constants.SuperstructureConstants.SCORE_CUBE_LOW.getElevatorPosition()),
          () -> Robot.gamePieceMode == GamePieceMode.CONE);
    }
  }
}
