package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

public class SwerveHeadingController {
  private static SwerveHeadingController instance;
  private Rotation2d setpoint;
  private PIDFFController controller;

  private SwerveHeadingController() {
    controller = new PIDFFController(DriveConstants.kHeadingControllerGains);
    controller.setTolerance(DriveConstants.kHeadingControllerGains.tolerance.get());
    controller.enableContinuousInput(-180, 180);
    setpoint = Robot.swerveDrive.getEstimatedPose().getRotation();
  }

  /**
   * Ensures the SwerveHeadingController is not created more than once.
   *
   * @return The SwerveHeadingController object.
   */
  public static SwerveHeadingController getInstance() {
    if (instance == null) {
      instance = new SwerveHeadingController();
    }

    return instance;
  }

  /**
   * Changes the setpoint of the heading controller. (Note that this value is not loaded into the
   * PID controller until update() is called.)
   *
   * @param setpoint The new setpoint of the heading controller.
   */
  public void setSetpoint(Rotation2d setpoint) {
    this.setpoint = setpoint;
  }

  public void addToSetpoint(Rotation2d setpoint) {
    this.setpoint = this.setpoint.plus(setpoint);
  }

  public Rotation2d getSetpoint() {
    return setpoint;
  }

  /**
   * Updates the heading controller PID with the setpoint and calculates output.
   *
   * @return The speed, in degrees per second, of rotation.
   */
  public double update() {
    if (Constants.tuningMode) {
      setSetpoint(
          Rotation2d.fromDegrees(
              SmartDashboard.getNumber(
                  "Heading Controller/setpoint degrees", setpoint.getDegrees())));
    } else {
      Logger.getInstance()
          .recordOutput("Heading Controller/setpoint degrees", setpoint.getDegrees());
    }

    SmartDashboard.putBoolean("Heading Controller/at setpoint", controller.atSetpoint());

    controller.setSetpoint(setpoint.getDegrees());
    Logger.getInstance().recordOutput("Heading Controller/setpoint", setpoint.getDegrees());
    double output = 0;
    if (!controller.atSetpoint()) {
      Rotation2d currentHeading = Robot.swerveDrive.getEstimatedPose().getRotation();
      output = controller.calculate(currentHeading.getDegrees(), setpoint.getDegrees());
      Logger.getInstance()
          .recordOutput(
              "Heading Controller/error", currentHeading.getDegrees() - setpoint.getDegrees());
    }

    Logger.getInstance().recordOutput("Heading Controller/update", output);
    return output;
  }
}
