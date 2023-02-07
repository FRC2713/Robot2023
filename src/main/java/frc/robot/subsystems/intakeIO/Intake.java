package frc.robot.subsystems.intakeIO;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Robot;
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
    IO.setVoltageRollers(rpm / (IntakeConstants.MAX_ROLLER_RPM) * 12);
  }

  public void setRollerRPM(double rpm) {
    this.targetRPM = rpm;
    IO.setVoltageWheels(rpm); // PLACEHOLDER VALUE
  }

  public void periodic() {

    IO.updateInputs(inputs);

    Logger.getInstance().recordOutput("Intake/Target RPM", targetRPM);
    Logger.getInstance().recordOutput("Intake/Has reached target", isAtTarget());

    Logger.getInstance().processInputs("Intake", inputs);
  }

  public static class Commands {

    public static Command cmdSetWheelVelocityRPM(double targetRPM) {
      return new InstantCommand(() -> Robot.intake.setWheelRpm(targetRPM));
    }

    public static Command cmdSetRollerVelocityRPM(double targetRPM) {
      return new InstantCommand(() -> Robot.intake.setRollerRPM(targetRPM));
    }

    public static Command cmdSetWheelVelocityRPMAndWait(double targetRPM) {
      return cmdSetWheelVelocityRPM(targetRPM).repeatedly().until(() -> Robot.intake.isAtTarget());
    }

    public static Command cmdSetRollerVelocityRPMAndWait(double targetRPM) {
      return cmdSetRollerVelocityRPM(targetRPM).repeatedly().until(() -> Robot.intake.isAtTarget());
    }
  }
}
