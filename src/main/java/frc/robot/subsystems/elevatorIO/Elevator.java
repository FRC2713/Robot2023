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
    this.feedforward = new ElevatorFeedforward(0, 0.545 + Math.PI, 0);
    this.elevatorController = new ProfiledPIDController(0.01, 0, 0, new Constraints(160, 500));
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

  public void setHeightBottomScore(){
    this.targetHeight = Constants.ElevatorConstants.ELEVATOR_LOW_SCORE;
  }
  public void setHeightConeMidScore(){
    this.targetHeight = Constants.ElevatorConstants.ELEVATOR_CONE_MID_SCORE;
  }
  public void setHeightCubeMidScore(){
    this.targetHeight = Constants.ElevatorConstants.ELEVATOR_CUBE_MID_SCORE;
  }
  public void setHeightConeHighScore(){
    this.targetHeight = Constants.ElevatorConstants.ELEVATOR_CONE_HIGH_SCORE;
  }
  public void setHeightCubeHighScore(){
    this.targetHeight = Constants.ElevatorConstants.ELEVATOR_CUBE_HIGH_SCORE;
  }
  public Command cmdSetTargetHeight(double targetHeightInches) {
    return new InstantCommand(() -> Robot.ele.setTargetHeight(targetHeightInches));
  }

  public Command cmdSetTargetHeightAndWait(double targetHeightInches) {
    return cmdSetTargetHeight(targetHeightInches).repeatedly().until(() -> atTargetHeight());
  }

  public double getCurrentHeight() {
    return inputs.heightInches;
  }

  public boolean atTargetHeight() {
    return Math.abs(getCurrentHeight() - targetHeight) < 1;
  }

  public void periodic() {
    double effort = elevatorController.calculate(inputs.heightInches, targetHeight);
    double ffEffort = feedforward.calculate(elevatorController.getSetpoint().velocity);
    effort += ffEffort;
    effort = MathUtil.clamp(effort, -12, 12);

    IO.updateInputs(inputs);
    IO.setVoltage(effort);
    Logger.getInstance().recordOutput("Elevator/Target Height", targetHeight);
    Logger.getInstance().recordOutput("Elevator/Control Effort", effort);
    Logger.getInstance().recordOutput("Elevator/FF Effort", ffEffort);
    Logger.getInstance().recordOutput("Elevator/isAtTarget", atTargetHeight());

    Logger.getInstance().processInputs("Elevator", inputs);
  }

  public static class Commands {
    public static Command setToHeight(double height) {
      return new InstantCommand(() -> Robot.ele.setTargetHeight(height), Robot.ele);
    }
  }
}
