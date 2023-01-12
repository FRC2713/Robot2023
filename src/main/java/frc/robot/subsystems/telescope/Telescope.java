package frc.robot.subsystems.telescope;

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
  //private final TelescopeInputsAutoLogged inputs;
  private double targetExtension = 0.0;
  private final SimpleMotorFeedforward feedforward;

  public Telescope(TelescopeIO IO) {
    this.feedforward = new SimpleMotorFeedforward(0, 0, 0);
    this.telescopeController =
        new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(200, 600));
    SmartDashboard.putData("Telescope PID", telescopeController);
    //this.inputs = new TelescopeInputsAutoLogged();
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
    //return inputs.heightInches;
    return 0;
  }

  public void Periodic() {
    //double effort = telescopeController.calculate(inputs);
  }
}
