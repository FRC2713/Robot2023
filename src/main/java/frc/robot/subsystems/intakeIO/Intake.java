package frc.robot.subsystems.intakeIO;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
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

  public void setRpm(double rpm) {
    this.targetRPM = rpm;
    IO.setVoltage(rpm / (IntakeConstants.MAX_ROLLER_RPM) * 12);
  }

  public double getCurrentDraw() {
    return inputs.wheelsCurrentAmps + inputs.rollersCurrentAmps;
  }

  public Command cmdSetVelocityRPM(double targetRPM) {
    return new InstantCommand(() -> setRpm(targetRPM));
  }

  public Command cmdSetVelocityRPMAndWait(double targetRPM) {
    return cmdSetVelocityRPM(targetRPM).repeatedly().until(() -> isAtTarget());
  }

  public void periodic() {

    IO.updateInputs(inputs);

    Logger.getInstance().recordOutput("Intake/Target RPM", targetRPM);
    Logger.getInstance().recordOutput("Intake/Has reached target", isAtTarget());

    Logger.getInstance().processInputs("Intake", inputs);
  }
}
