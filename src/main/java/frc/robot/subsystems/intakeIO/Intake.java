package frc.robot.subsystems.intakeIO;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.Robot;
import frc.robot.Robot.GamePieceMode;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO IO;
  private final IntakeInputsAutoLogged inputs;
  private double targetRPM = 0.0;

  public Intake(IntakeIO IO) {
    this.inputs = new IntakeInputsAutoLogged();
    IO.updateInputs(inputs);
    this.IO = IO;
  }

  public boolean isAtTarget() {
    return Math.abs(inputs.rollersVelocityRPM - targetRPM)
        < 0.5; // might be wheel rpm ¯\_(ツ)_/¯ we shall see
  }

  public void setWheelRpm(double rpm) {
    this.targetRPM = rpm;
    IO.setVoltageRollers(rpm / (IntakeConstants.MAX_WHEEL_RPM) * 12);
  }

  public void setRollerRPM(double rpm) {
    this.targetRPM = rpm;
    IO.setVoltageWheels(rpm / (IntakeConstants.MAX_ROLLER_RPM) * 12); // PLACEHOLDER VALUE
  }

  public double getCurrentDraw() {
    return inputs.wheelsCurrentAmps + inputs.rollersCurrentAmps;
  }

  public void periodic() {

    IO.updateInputs(inputs);

    Logger.getInstance().recordOutput("Intake/Target RPM", targetRPM);
    Logger.getInstance().recordOutput("Intake/Has reached target", isAtTarget());

    Logger.getInstance().processInputs("Intake", inputs);
  }

  public static class Commands {

    public static Command setWheelVelocityRPM(double targetRPM) {
      return new InstantCommand(() -> Robot.intake.setWheelRpm(targetRPM));
    }

    public static Command setRollerVelocityRPM(double targetRPM) {
      return new InstantCommand(() -> Robot.intake.setRollerRPM(targetRPM));
    }

    public static Command setWheelVelocityRPMAndWait(double targetRPM) {
      return setWheelVelocityRPM(targetRPM).repeatedly().until(() -> Robot.intake.isAtTarget());
    }

    public static Command setRollerVelocityRPMAndWait(double targetRPM) {
      return setRollerVelocityRPM(targetRPM).repeatedly().until(() -> Robot.intake.isAtTarget());
    }

    public static Command score() {
      return new ConditionalCommand(
          new ParallelCommandGroup(
              setRollerVelocityRPM(SuperstructureConstants.SCORE_CUBE_MID.getRollerRPM()),
              setWheelVelocityRPM(SuperstructureConstants.SCORE_CUBE_MID.getWheelRPM())),
          new ParallelCommandGroup(
              setRollerVelocityRPM(SuperstructureConstants.SCORE_CONE_MID.getRollerRPM()),
              setWheelVelocityRPM(SuperstructureConstants.SCORE_CONE_MID.getWheelRPM())),
          () -> Robot.gamePieceMode == GamePieceMode.CUBE);
    }
  }
}
