package frc.robot.subsystems.intakeIO;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class IntakeIOSim implements IntakeIO {
  private final DCMotorSim simWheels =
      new DCMotorSim(
          DCMotor.getNeo550(1), Constants.IntakeConstants.GEARING, Constants.IntakeConstants.MOI);
  private final DCMotorSim simRollers =
      new DCMotorSim(
          DCMotor.getNeo550(1), Constants.IntakeConstants.GEARING, Constants.IntakeConstants.MOI);

  @Override
  public void updateInputs(IntakeInputs inputs) {
    if (DriverStation.isDisabled()) {
      simWheels.setInputVoltage(0.0);
      simRollers.setInputVoltage(0.0);
    }
    simWheels.update(0.02);
    simRollers.update(0.02);
    inputs.wheelsOutputVoltage = MathUtil.clamp(simWheels.getOutput(0), -12.0, 12.0);
    inputs.rollersOutputVoltage = MathUtil.clamp(simRollers.getOutput(0), -12.0, 12.0);
    inputs.wheelsIsOn = simWheels.getAngularVelocityRadPerSec() > 0.005;
    inputs.rollersIsOn = simRollers.getAngularVelocityRadPerSec() > 0.005;
    inputs.wheelsVelocityRPM = simWheels.getAngularVelocityRPM();
    inputs.rollersVelocityRPM = simRollers.getAngularVelocityRPM();
    inputs.wheelsTempCelcius = 0.0;
    inputs.rollersTempCelcius = 0.0;
    inputs.wheelsCurrentAmps = simWheels.getCurrentDrawAmps();
    inputs.rollersCurrentAmps = simRollers.getCurrentDrawAmps();
    inputs.wheelsPositionRad = simWheels.getAngularPositionRad();
    inputs.rollersPositionRad = simRollers.getAngularPositionRad();
  }

  @Override
  public void setVoltageWheels(double volts) {
    simWheels.setInputVoltage(volts);
  }

  @Override
  public void setVoltageRollers(double volts) {
    simRollers.setInputVoltage(volts);
  }
}
