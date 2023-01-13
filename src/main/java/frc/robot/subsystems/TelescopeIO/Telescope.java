package frc.robot.subsystems.TelescopeIO;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Telescope extends SubsystemBase {

  private final ProfiledPIDController telescopeController;
  private final TelescopeInputsAutoLogged inputs;
  private final TelescopeIO IO;
  private double targetHeight = 0.0;
  private final SimpleMotorFeedforward feedForward;

  public Telescope(TelescopeIO IO){
    this.feedForward = new SimpleMotorFeedforward(0, 1, 2);
    this.telescopeController = new ProfiledPIDController(0, 0, 0, new Constraints(100, 200));
    SmartDashboard.putData("Telescope PID", telescopeController);
    this.inputs = new TelescopeInputsAutoLogged();
    IO.updateInputs(inputs);
    this.IO = IO;
  }

  public void setTargetHeight(double targetHeightInches){

    if(targetHeightInches > Units.metersToInches(Constants.Telescope.TELESCOPE_MAX_HEIGHT_METERS)){
        Logger.getInstance().recordMetadata("Telescope/ERRORS", "Target Height Overextended");
        this.targetHeight = 
            MathUtil.clamp(targetHeightInches,
            0,
            Units.metersToInches(Constants.Telescope.TELESCOPE_MAX_HEIGHT_METERS));
            return;
    }   
    this.targetHeight = targetHeightInches;
  }

  public double getCurrentHeight(){
    return inputs.heightInches;
  }

  public void periodic(){
    double effort = telescopeController.calculate(inputs.heightInches, targetHeight);
    double ffEffort = feedForward.calculate(telescopeController.getSetpoint().velocity);
    effort += ffEffort;
    effort = MathUtil.clamp(effort, 0, 0);
    IO.updateInputs(inputs);
    IO.setVoltage(effort);
    Logger.getInstance().recordOutput("Telescope/Target Height", targetHeight);
    Logger.getInstance().recordOutput("Telescope/Control Effort", effort);
    Logger.getInstance().recordOutput("Telescope/FF Effort", ffEffort);
    Logger.getInstance().processInputs("Telescope", inputs);
  }
}