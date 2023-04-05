package frc.robot.subsystems.slapperIO;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Slapper extends SubsystemBase {
  private final SlapperIO IO;
  private final SlapperInputsAutoLogged inputs;
  private double targetangleDeg = 90.0;
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
      return Math.abs(inputs.positionDeg - targetangleDeg) < 10;
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
      effort = controller.calculate(inputs.positionDeg, targetangleDeg);
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

  public double getPositionDeg() {
    return inputs.positionDeg;
  }

  // public static class Commands {
  //   public static Command sendItAndWait() {
  //     return new SequentialCommandGroup(
  //         new InstantCommand(
  //             () -> {
  //               Robot.slapper.usePid = false;
  //               Robot.slapper.setTarget(Constants.SlapperConstants.FULL_SEND_DEG);
  //             }),
  //         new WaitUntilCommand(() -> Robot.slapper.isAtTarget()));
  //   }

  //   public static Command comeBackHome() {
  //     return new InstantCommand(
  //         () -> {
  //           Robot.slapper.usePid = true;
  //           Robot.slapper.setTarget(Constants.SlapperConstants.RESTING_DEG);
  //         });
  //   }
  // }
}
