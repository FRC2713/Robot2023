package frc.robot.subsystems.intakeIO;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.Robot;
import frc.robot.Robot.GamePieceMode;
import frc.robot.subsystems.LightStrip.Pattern;
import frc.robot.util.LoggableMotor;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO IO;
  private final IntakeInputsAutoLogged inputs;
  private double targetRPM = 0.0;
  private double cubeDetectionThreshold = 0.4;
  private double coneDetectionThreshold = 0.25;
  private double filteredVoltageCube = 0, filteredVoltageCone;
  public boolean scoring = false;

  private boolean previouslyHadGamePiece = false;

  private Timer timer = new Timer();
  private Debouncer debouncer = new Debouncer(0.1);
  private LinearFilter analogVoltageFilterRight = LinearFilter.singlePoleIIR(0.04, 0.02);
  private LinearFilter analogVoltageFilterLeft = LinearFilter.singlePoleIIR(0.04, 0.02);
  private LoggableMotor topRollerMotor, bottomRollerMotor;

  public Intake(IntakeIO IO) {
    this.inputs = new IntakeInputsAutoLogged();
    IO.updateInputs(inputs);
    this.IO = IO;
    topRollerMotor = new LoggableMotor("Intake Top Roller", DCMotor.getNeo550(1));
    bottomRollerMotor = new LoggableMotor("Intake Bottom Roller", DCMotor.getNeo550(1));
  }

  public boolean isAtTarget() {
    return Math.abs(inputs.topVelocityRPM - targetRPM) < 0.5;
  }

  public void setTopRPM(double rpm) {
    Logger.getInstance()
        .recordOutput("Intake/Applied Top Volts", rpm / (IntakeConstants.MAX_TOP_RPM) * 12);
    IO.setTopVoltage(rpm / (IntakeConstants.MAX_TOP_RPM) * 12);
  }

  public void setBottomRPM(double rpm) {
    Logger.getInstance()
        .recordOutput("Intake/Applied Bottom Volts", rpm / (IntakeConstants.MAX_BOTTOM_RPM) * 12);
    IO.setBottomVoltage(rpm / (IntakeConstants.MAX_BOTTOM_RPM) * 12); // PLACEHOLDER VALUE
  }

  public double getCurrentDraw() {
    return inputs.topCurrentAmps + inputs.bottomCurrentAmps;
  }

  public boolean hasGamepiece() {
    if (Robot.gamePieceMode == GamePieceMode.CUBE) {
      return (filteredVoltageCube > cubeDetectionThreshold) && !scoring;
    } else {
      return debouncer.calculate(filteredVoltageCone > coneDetectionThreshold) && !scoring;
    }
  }

  public void setScoring(boolean scoring) {
    this.scoring = scoring;
  }

  public void periodic() {
    IO.updateInputs(inputs);
    topRollerMotor.log(inputs.topCurrentAmps, inputs.topOutputVoltage);
    bottomRollerMotor.log(inputs.bottomCurrentAmps, inputs.bottomOutputVoltage);

    filteredVoltageCube = analogVoltageFilterRight.calculate(inputs.encoderVoltageRight);
    filteredVoltageCone = analogVoltageFilterLeft.calculate(inputs.encoderVoltageLeft);

    Logger.getInstance().recordOutput("Intake/Filtered Sensor Cube", filteredVoltageCube);
    Logger.getInstance().recordOutput("Intake/Filtered Sensor Cone", filteredVoltageCone);

    Logger.getInstance().recordOutput("Intake/Target RPM", targetRPM);
    Logger.getInstance().recordOutput("Intake/Has reached target", isAtTarget());

    Logger.getInstance().processInputs("Intake", inputs);
    Logger.getInstance().recordOutput("Intake/Scoring", scoring);
    Logger.getInstance().recordOutput("Intake/Has gamepiece", hasGamepiece());

    if (hasGamepiece() && Robot.gamePieceMode == GamePieceMode.CUBE) {
      setTopRPM(SuperstructureConstants.HOLD_CUBE.getTopRPM());
      setBottomRPM(SuperstructureConstants.HOLD_CUBE.getBottomRPM());
    }

    if (hasGamepiece() && Robot.gamePieceMode == GamePieceMode.CONE) {
      setTopRPM(SuperstructureConstants.HOLD_CONE.getTopRPM());
      setBottomRPM(SuperstructureConstants.HOLD_CONE.getBottomRPM());
    }

    if (hasGamepiece() && !previouslyHadGamePiece) {
      timer.restart();
      previouslyHadGamePiece = true;
      // RumbleManager.getInstance().setDriver(1.0, 2.0);
      if (timer.get() <= 2.0) {
        Robot.lights.setColorPattern(Pattern.DarkGreen);
      }
    }

    if (!hasGamepiece()) {
      previouslyHadGamePiece = !true;
    }
  }

  public void setCurrentLimit(int currentLimit) {
    IO.setCurrentLimit(currentLimit);
  }

  public static class Commands {

    public static Command setTopVelocityRPM(double targetRPM) {
      return new InstantCommand(() -> Robot.intake.setTopRPM(targetRPM));
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
          new ConditionalCommand(
              new ParallelCommandGroup(
                  setTopVelocityRPM(SuperstructureConstants.SCORE_CUBE_MID.getTopRPM()),
                  setBottomVelocityRPM(SuperstructureConstants.SCORE_CUBE_MID.getBottomRPM())),
              new ParallelCommandGroup(
                  setTopVelocityRPM(SuperstructureConstants.SCORE_CUBE_LOW.getTopRPM()),
                  setBottomVelocityRPM(SuperstructureConstants.SCORE_CUBE_LOW.getBottomRPM())),
              () -> Robot.elevator.getTargetHeight() > 8),
          new ParallelCommandGroup(
              setTopVelocityRPM(SuperstructureConstants.SCORE_CONE_MID.getTopRPM()),
              setBottomVelocityRPM(SuperstructureConstants.SCORE_CONE_MID.getBottomRPM())),
          () -> Robot.gamePieceMode == GamePieceMode.CUBE);
    }
  }
}
