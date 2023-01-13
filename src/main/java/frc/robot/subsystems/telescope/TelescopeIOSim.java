package frc.robot.subsystems.telescope;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class TelescopeIOSim implements TelescopeIO {
  private final Mechanism2d mech;
  private final MechanismRoot2d root;
  private final MechanismLigament2d m_telescope;

  public TelescopeIOSim() {
    mech = new Mechanism2d(0, 0);
    root = mech.getRoot("climber", 0, 0);
    m_telescope = root.append(new MechanismLigament2d("telescope", 0, 0));
    SmartDashboard.putData("Telescope Mech2D", mech);
  }

  private ElevatorSim teleSim =
      new ElevatorSim(
          DCMotor.getNEO(2),
          1,
          Constants.Telescope.CARRIAGE_MASS_KG,
          Constants.Telescope.TELESCOPE_DRUM_RADIUS_METERS,
          Constants.Telescope.TELESCOPE_MIN_EXTENSION,
          Constants.Telescope.TELESCOPE_MAX_EXTENSION,
          false);

  @Override
  public void updateInputs(TelescopeInputs inputs) {
    if (DriverStation.isDisabled()) {
      return;
    }
    teleSim.update(0.02);
    inputs.outputVoltage = MathUtil.clamp(teleSim.getOutput(0), 0, 0);
    inputs.extendedInches = Units.metersToInches(teleSim.getPositionMeters());
    inputs.velocityInchesPerSecond = Units.metersToInches(teleSim.getVelocityMetersPerSecond());
    inputs.tempCelsius = 0.0;

    m_telescope.setLength(0 + inputs.extendedInches);
  }

  @Override
  public void setVoltage(double volts) {
    teleSim.setInputVoltage(volts);
  }
}
