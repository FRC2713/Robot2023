package frc.robot.subsystems.swerveIO;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.RobotMap;

public class SwerveIOPigeon2 implements SwerveIO {

  private final Pigeon2 gyro;

  public SwerveIOPigeon2() {
    gyro = new Pigeon2(RobotMap.pigeonCANId);
    gyro.zeroGyroBiasNow();
    gyro.setYaw(0);
  }

  @Override
  public void updateInputs(SwerveInputs inputs) {
    inputs.gyroCompassHeading = gyro.getAbsoluteCompassHeading();
    inputs.gyroPitchPosition = gyro.getPitch();
    inputs.gyroRollPosition = gyro.getRoll();
    inputs.gyroYawPosition = gyro.getYaw() - 180; // gyro faces backwards on the robot
  }

  @Override
  public void resetGyro(Rotation2d rotation2d) {
    gyro.setYaw(rotation2d.getDegrees());
  }
}
