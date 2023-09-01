package frc.robot.subsystems.swerveIO.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Robot;
import frc.robot.subsystems.swerveIO.SwerveIO;

public class SwerveIOSim implements SwerveIO {

  private final OdometryToGyroAdapter notAGyro = new OdometryToGyroAdapter();
  private double[] yprDegrees = new double[3];

  @Override
  public void updateInputs(SwerveInputs inputs) {
    inputs.previousgyroPitchPosition = inputs.gyroPitchPosition;

    notAGyro.getYPR(yprDegrees);

    inputs.gyroCompassHeading = 0;
    inputs.gyroYawPosition = yprDegrees[0];
    inputs.gyroPitchPosition = yprDegrees[1];
    inputs.gyroRollPosition = yprDegrees[2];

    // inputs.gyroYawPosition =
    //     SwerveSubsystem.resetGyroVal == null ? 0 : SwerveSubsystem.resetGyroVal.getDegrees();
  }

  public void zeroGyro() {}

  @Override
  public void resetGyro(Rotation2d rotation2d) {
    // SwerveSubsystem.resetGyroVal = rotation2d;
  }

  @Override
  public void initalizeGyroSource() {
    notAGyro.setSources(Robot.swerveDrive.getOdometry(), Robot.swerveDrive.getModules());
  }
}
