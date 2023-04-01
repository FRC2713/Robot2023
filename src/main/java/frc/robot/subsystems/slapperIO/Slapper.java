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
  private double targetangleDeg = 0.0;
  public PIDController controller;
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
    return Math.abs(inputs.velocityRPM - targetangleDeg) < 0.5;
  }

  public void setTarget(double angleDeg) {
    targetangleDeg = angleDeg;
  }

  public double getCurrentDraw() {
    return inputs.currentAmps;
  }

  public void periodic() {

    double effort =
        controller.calculate(
            Units.degreesToRadians(inputs.positionDeg), Units.degreesToRadians(targetangleDeg));
    effort += FF.calculate(Units.degreesToRadians(inputs.positionDeg), inputs.velocityRPM);
    effort = MathUtil.clamp(effort, -12, 12);

    Logger.getInstance().recordOutput("Slapper/Effort", effort);
    Logger.getInstance().recordOutput("Slapper/Target", targetangleDeg);
    Logger.getInstance().recordOutput("Slapper/Position", inputs.positionDeg);

    // Logger.getInstance()
    //     .recordOutput("Slapper/Setpoint Velocity", controller.getSetpoint().velocity);

    // Logger.getInstance()
    //     .recordOutput("Slapper/Setpoint Position", controller.getSetpoint().position);

    IO.setVoltage(effort);
    IO.updateInputs(inputs);
  }

  public void setCurrentLimit(int currentLimit) {
    IO.setCurrentLimit(currentLimit);
  }

  public static class Commands {}
}
