package frc.robot.subsystems.slapper;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Slapper extends SubsystemBase {
  private CANSparkMax slapperMotor;
  private int canID = 100;
  private double targetAngleDegrees = Constants.SlapperConstants.MIN_ANGLE_DEG;
  public boolean isAtTarget;
  private SingleJointedArmSim slapperSim;

  public Slapper() {
    // deviceID, MotorType
    // slapperMotor = new CANSparkMax(canID, MotorType.kBrushless);
    slapperSim =
        new SingleJointedArmSim(
            Constants.SlapperConstants.MOTOR,
            Constants.SlapperConstants.GEARING,
            Constants.SlapperConstants.MOI,
            Constants.SlapperConstants.LENGTH_METRES,
            Units.degreesToRadians(Constants.SlapperConstants.MIN_ANGLE_DEG),
            Units.degreesToRadians(Constants.SlapperConstants.MAX_ANGLE_DEG),
            false);
  }

  public void goToAngle(double angle) {
    targetAngleDegrees = angle;
  }

  // Runs every 20 ms
  public void periodic() {
    // Encoder Position = 0
    double pos = slapperSim.getAngleRads();
    pos = Units.radiansToDegrees(pos);
    // 0 * 360 = 0
    // pos *= 360;
    // 0 - 90 = -90
    double voltage = targetAngleDegrees - pos;
    // -90 / 10 = -9
    voltage *= Constants.SlapperConstants.DEMO_GAINS;
    // Send it!
    // slapperMotor.setVoltage(voltage);

    isAtTarget = Math.abs(pos - targetAngleDegrees) < 0.1;

    slapperSim.update(0.02);
    slapperSim.setInputVoltage(voltage);

    Logger.getInstance().recordOutput("Slapper/Voltage", voltage);
  }

  public double getPositionDeg() {
    return Units.radiansToDegrees(slapperSim.getAngleRads());
  }
}
