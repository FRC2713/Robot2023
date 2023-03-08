package frc.robot.subsystems.swerveIO;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.MotionHandler.MotionMode;

public class SwerveIOSim implements SwerveIO {

  public static MotionMode motionMode = MotionMode.FULL_DRIVE;

  @Override
  public void updateInputs(SwerveInputs inputs) {
    inputs.gyroCompassHeading = 0;
    inputs.gyroPitchPosition = 0;
    inputs.previousgyroPitchPosition = 0;
    inputs.gyroRollPosition = 0;
    inputs.gyroYawPosition =
        SwerveSubsystem.resetGyroVal == null ? 0 : SwerveSubsystem.resetGyroVal.getDegrees();
  }

  public void zeroGyro() {}

  @Override
  public void resetGyro(Rotation2d rotation2d) {
    SwerveSubsystem.resetGyroVal = rotation2d;
  }
}
