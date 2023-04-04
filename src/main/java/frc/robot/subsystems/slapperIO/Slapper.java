package frc.robot.subsystems.slapperIO;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

public class Slapper extends SubsystemBase {
  private final SlapperIO IO;
  private final SlapperInputsAutoLogged inputs;
  private double targetangleDeg = 0.0;
  public PIDController controller;
  public boolean usePid = true;
  private final ArmFeedforward FF;
  public boolean scoring = false;

  public Slapper(SlapperIO IO) {
    this.controller = Constants.SlapperConstants.GAINS.createWpilibController();
    this.FF = Constants.SlapperConstants.GAINS.createArmFeedforward();
    this.inputs = new SlapperInputsAutoLogged();
    IO.updateInputs(inputs);
    this.IO = IO;
  }

  public boolean isAtTarget() {
    if (!usePid) {
      if (Math.signum(targetangleDeg) == -1) {
        return inputs.positionDeg <= targetangleDeg;
      }
      return inputs.positionDeg >= targetangleDeg;
    }
    return Math.abs(inputs.positionDeg - targetangleDeg) < 0.5;
  }

  public void setTarget(double angleDeg) {
    targetangleDeg = angleDeg;
  }

  public double getCurrentDraw() {
    return inputs.currentAmps;
  }

  public void periodic() {
    double effort;

    if (usePid) {
      effort =
          controller.calculate(
              Units.degreesToRadians(inputs.positionDeg), Units.degreesToRadians(targetangleDeg));
    } else {
      effort = !isAtTarget() ? (targetangleDeg - inputs.positionDeg) * 0.1 : 0;
    }

    effort += FF.calculate(Units.degreesToRadians(inputs.positionDeg), inputs.velocityRPM);
    effort = MathUtil.clamp(effort, -12, 12);

    Logger.getInstance().recordOutput("Slapper/Effort", effort);
    Logger.getInstance().recordOutput("Slapper/Target", targetangleDeg);
    Logger.getInstance().recordOutput("Slapper/isAtTarget", isAtTarget());
    Logger.getInstance().recordOutput("Slapper/usingPID", usePid);

    IO.setVoltage(effort);
    IO.updateInputs(inputs);
    Logger.getInstance().processInputs("Slapper", inputs);
  }

  public void setCurrentLimit(int currentLimit) {
    IO.setCurrentLimit(currentLimit);
  }

  public static class Commands {
    public static Command sendItAndWait() {
      return new SequentialCommandGroup(
          new InstantCommand(
              () -> {
                Robot.slapper.usePid = false;
                Robot.slapper.setTarget(Constants.SlapperConstants.FULL_SEND_DEG);
              }),
          new WaitUntilCommand(() -> Robot.slapper.isAtTarget()),
          new InstantCommand(
              () -> {
                Robot.slapper.usePid = true;
              }));
    }

    public static Command comeBackHome() {
      return new InstantCommand(
          () -> {
            Robot.slapper.usePid = true;
            Robot.slapper.setTarget(Constants.SlapperConstants.RESTING_DEG);
          });
    }
  }
}
