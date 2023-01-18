package frc.robot.subsystems.swerveIO;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveIOSim implements SwerveIO {

  @Override
  public void updateInputs(SwerveInputs inputs) {
    inputs.gyroCompassHeading = 0;
    inputs.gyroPitchPosition = 0;
    inputs.gyroRollPosition = 0;
    inputs.gyroYawPosition = 0;
  }

  @Override
  public void resetGyro(Rotation2d rotation2d) {}
}
