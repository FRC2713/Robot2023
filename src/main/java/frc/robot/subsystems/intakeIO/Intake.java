package frc.robot.subsystems.intakeIO;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.Robot;
import frc.robot.Robot.GamePieceMode;
import frc.robot.subsystems.LightStrip.Pattern;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO IO;
  private final IntakeInputsAutoLogged inputs;
  private double targetRPM = 0.0;
  private double detectionThreshold = 1.3;
  private double filteredVoltage = 0;
  public boolean scoring = false;

  private LinearFilter analogVoltageFilter = LinearFilter.singlePoleIIR(0.06, 0.02);

  public Intake(IntakeIO IO) {
    this.inputs = new IntakeInputsAutoLogged();
    IO.updateInputs(inputs);
    this.IO = IO;
  }

  public boolean isAtTarget() {
    return Math.abs(inputs.topVelocityRPM - targetRPM) < 0.5;
  }

  public void setTopRpm(double rpm) {
    IO.setTopVoltage(rpm / (IntakeConstants.MAX_TOP_RPM) * 12);
  }

  public void setBottomRPM(double rpm) {
    IO.setBottomVoltage(rpm / (IntakeConstants.MAX_BOTTOM_RPM) * 12); // PLACEHOLDER VALUE
  }

  public double getCurrentDraw() {
    return inputs.topCurrentAmps + inputs.bottomCurrentAmps;
  }

  public boolean hasGamepiece() {
    return (filteredVoltage > detectionThreshold) && !scoring;
  }

  public void setScoring(boolean scoring) {
    this.scoring = scoring;
  }

  public void periodic() {

    IO.updateInputs(inputs);

    filteredVoltage = analogVoltageFilter.calculate(inputs.encoderVoltage);

    Logger.getInstance().recordOutput("Intake/Filtered Sensor 1", filteredVoltage);

    Logger.getInstance().recordOutput("Intake/Target RPM", targetRPM);
    Logger.getInstance().recordOutput("Intake/Has reached target", isAtTarget());

    Logger.getInstance().processInputs("Intake", inputs);

    if (hasGamepiece()) {
      IO.setTopVoltage(Constants.zero);
      IO.setBottomVoltage(Constants.zero);
      Robot.lights.setColorPattern(Pattern.DarkGreen);
    }
  }

  public void setCurrentLimit(int currentLimit) {
    IO.setCurrentLimit(currentLimit);
  }

  public static class Commands {

    public static Command setTopVelocityRPM(double targetRPM) {
      return new InstantCommand(() -> Robot.intake.setTopRpm(targetRPM));
    }

    public static Command setBottomVelocityRPM(double targetRPM) {
      return new InstantCommand(() -> Robot.intake.setBottomRPM(targetRPM));
    }

    public static Command setTopVelocityRPMAndWait(double targetRPM) {
      return setTopVelocityRPM(targetRPM).repeatedly().until(() -> Robot.intake.isAtTarget());
    }

    public static Command setBottomVelocityRPMAndWait(double targetRPM) {
      return setBottomVelocityRPM(targetRPM).repeatedly().until(() -> Robot.intake.isAtTarget());
    }

    public static Command score() {
      return new ConditionalCommand(
          new ParallelCommandGroup(
              setTopVelocityRPM(SuperstructureConstants.SCORE_CUBE_MID.getTopRPM()),
              setBottomVelocityRPM(SuperstructureConstants.SCORE_CUBE_MID.getBottomRPM())),
          new ParallelCommandGroup(
              setTopVelocityRPM(SuperstructureConstants.SCORE_CONE_MID.getTopRPM()),
              setBottomVelocityRPM(SuperstructureConstants.SCORE_CONE_MID.getBottomRPM())),
          () -> Robot.gamePieceMode == GamePieceMode.CUBE);
    }
  }
}
