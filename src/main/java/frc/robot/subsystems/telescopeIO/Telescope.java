package frc.robot.subsystems.telescopeIO;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Telescope extends SubsystemBase {

  private final TelescopeIO IO;
  private final ProfiledPIDController telescopeController;
  private final TelescopeInputsAutoLogged inputs;
  private double targetExtension = 0.0;
  private final SimpleMotorFeedforward feedforward;

  public Telescope(TelescopeIO IO) {
    this.feedforward = new SimpleMotorFeedforward(0, 0, 0);
    this.telescopeController =
        new ProfiledPIDController(0.5, 0, 0.2, new TrapezoidProfile.Constraints(200, 600));
    SmartDashboard.putData("Telescope PID", telescopeController);
    this.inputs = new TelescopeInputsAutoLogged();
    this.IO = IO;
  }

  public void setTargetExtensionInches(double targetExtensionInches) {
    if (targetExtensionInches > Units.metersToInches(Constants.Telescope.TELESCOPE_MAX_EXTENSION)) {
      Logger.getInstance().recordOutput("Telescope/Errors", "Target extension over max");
      this.targetExtension =
          MathUtil.clamp(targetExtensionInches, 0, Constants.Telescope.TELESCOPE_MAX_EXTENSION);
      return;
    }
    this.targetExtension = targetExtensionInches;
  }

  public double getCurrentExtension() {
    return inputs.extendedInches;
  }

  public void periodic() {
    double effort = telescopeController.calculate(inputs.extendedInches, targetExtension);
    double ffEffort = feedforward.calculate(telescopeController.getSetpoint().velocity);
    effort += ffEffort;
    effort = MathUtil.clamp(effort, -12, 12);

    IO.updateInputs(inputs);
    IO.setVoltage(effort);
    Logger.getInstance().recordOutput("Telescope/Target Extension", targetExtension);
    Logger.getInstance().recordOutput("Telescope/Control Effort", effort);
    Logger.getInstance().recordOutput("Telescope/FF Effort", ffEffort);

    Logger.getInstance().processInputs("Telescope", inputs);
  }
}
