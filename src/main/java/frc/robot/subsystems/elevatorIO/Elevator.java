package frc.robot.subsystems.elevatorIO;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.RedHawkUtil;
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
            new Constraints(50, 30));
    SmartDashboard.putData("Elevator PID", elevatorController);
    this.inputs = new ElevatorInputsAutoLogged();
    IO.updateInputs(inputs);
    this.IO = IO;
  }

  public void setTargetHeight(double targetHeightInches) {
    if (targetHeightInches
        > Units.metersToInches(Constants.ElevatorConstants.ELEVATOR_MAX_HEIGHT_METERS)) {
      RedHawkUtil.ErrHandler.getInstance().addError("Target height too high");
      this.targetHeight =
          MathUtil.clamp(
              targetHeightInches,
              0,
              Units.metersToInches(Constants.ElevatorConstants.ELEVATOR_MAX_HEIGHT_METERS));
      return;
    }
    this.targetHeight = targetHeightInches;
  }

  public double getCurrentHeight() {
    return inputs.heightInchesLeft;
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
    double effortLeft = elevatorController.calculate(inputs.heightInchesLeft, targetHeight);
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

    // if (shouldResetEncoders()) {
    //   IO.resetEncoders();
    // }
  }

  // public void resetencoders() {
  //   IO.resetEncoders();
  // }

  // public boolean shouldResetEncoders() {
  //   var avgVelocity =
  //       (Math.abs(inputs.velocityInchesPerSecondLeft)
  //               + Math.abs(inputs.velocityInchesPerSecondRight))
  //           / 2.0;
  //   var avgCurrent = (inputs.currentDrawAmpsLeft + inputs.currentDrawAmpsRight) / 2.0;
  //   var avgHeight = (Math.abs(inputs.heightInchesLeft) + Math.abs(inputs.heightInchesRight)) /
  // 2.0;
  //   return (Math.abs(avgVelocity) < 0.01)
  //       && avgCurrent > 3.0 /* might need to tune */
  //       && avgHeight < 2.0;
  // }

  public static class Commands {

    public static Command setTargetHeightAndWait(double targetHeightInches) {
      return setToHeight(targetHeightInches)
          .repeatedly()
          .until(() -> Robot.elevator.atTargetHeight());
    }

    public static Command setToHeight(double height) {
      return new InstantCommand(() -> Robot.elevator.setTargetHeight(height), Robot.elevator);
    }

    public static Command elevatorConeLowScore() {
      return new InstantCommand(
          () -> Robot.elevator.setTargetHeight(Constants.ElevatorConstants.ELEVATOR_CONE_LOW_SCORE),
          Robot.elevator);
    }

    public static Command elevatorConeLowScoreAndWait() {
      return setTargetHeightAndWait(Constants.ElevatorConstants.ELEVATOR_CONE_LOW_SCORE);
    }

    public static Command elevatorCubeLowScore() {
      return new InstantCommand(
          () -> Robot.elevator.setTargetHeight(Constants.ElevatorConstants.ELEVATOR_CUBE_LOW_SCORE),
          Robot.elevator);
    }

    public static Command elevatorCubeLowScoreAndWait() {
      return setTargetHeightAndWait(Constants.ElevatorConstants.ELEVATOR_CUBE_LOW_SCORE);
    }

    public static Command elevatorConeMidScore() {
      return new InstantCommand(
          () -> Robot.elevator.setTargetHeight(Constants.ElevatorConstants.ELEVATOR_CONE_MID_SCORE),
          Robot.elevator);
    }

    public static Command elevatorConeMidScoreAndWait() {
      return setTargetHeightAndWait(Constants.ElevatorConstants.ELEVATOR_CONE_MID_SCORE);
    }

    public static Command elevatorCubeMidScore() {
      return new InstantCommand(
          () -> Robot.elevator.setTargetHeight(Constants.ElevatorConstants.ELEVATOR_CUBE_MID_SCORE),
          Robot.elevator);
    }

    public static Command elevatorCubeMidScoreAndWait() {
      return setTargetHeightAndWait(Constants.ElevatorConstants.ELEVATOR_CUBE_MID_SCORE);
    }

    public static Command elevatorConeHighScore() {
      return new InstantCommand(
          () ->
              Robot.elevator.setTargetHeight(Constants.ElevatorConstants.ELEVATOR_CONE_HIGH_SCORE),
          Robot.elevator);
    }

    public static Command elevatorConeHighScoreAndWait() {
      return setTargetHeightAndWait(Constants.ElevatorConstants.ELEVATOR_CONE_HIGH_SCORE);
    }

    public static Command elevatorCubeHighScore() {
      return new InstantCommand(
          () ->
              Robot.elevator.setTargetHeight(Constants.ElevatorConstants.ELEVATOR_CUBE_HIGH_SCORE),
          Robot.elevator);
    }

    public static Command elevatorCubeHighScoreAndWait() {
      return setTargetHeightAndWait(Constants.ElevatorConstants.ELEVATOR_CUBE_HIGH_SCORE);
    }

    public static Command elevatorCubeFloorIntake() {
      return new InstantCommand(
          () ->
              Robot.elevator.setTargetHeight(
                  Constants.ElevatorConstants.ELEVATOR_CUBE_FLOOR_INTAKE),
          Robot.elevator);
    }

    public static Command elevatorCubeFloorIntakeAndWait() {
      return setTargetHeightAndWait(Constants.ElevatorConstants.ELEVATOR_CUBE_FLOOR_INTAKE);
    }

    public static Command elevatorConeFloorTippedIntake() {
      return new InstantCommand(
          () ->
              Robot.elevator.setTargetHeight(
                  Constants.ElevatorConstants.ELEVATOR_CONE_FLOOR_TIPPED_INTAKE),
          Robot.elevator);
    }

    public static Command elevatorConeFloorTippedIntakeAndWait() {
      return setTargetHeightAndWait(Constants.ElevatorConstants.ELEVATOR_CONE_FLOOR_TIPPED_INTAKE);
    }

    public static Command elevatorConeFloorUpIntake() {
      return new InstantCommand(
          () ->
              Robot.elevator.setTargetHeight(
                  Constants.ElevatorConstants.ELEVATOR_CONE_FLOOR_UP_INTAKE),
          Robot.elevator);
    }

    public static Command elevatorConeFloorUpIntakeAndWait() {
      return setTargetHeightAndWait(Constants.ElevatorConstants.ELEVATOR_CONE_FLOOR_UP_INTAKE);
    }

    public static Command elevatorCurrentHeight() {
      return new InstantCommand(
          () -> Robot.elevator.setTargetHeight(Robot.elevator.getCurrentHeight()), Robot.elevator);
    }
  }
}
