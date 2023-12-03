package lib.mechadv;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.util.ErrorTracker;
import lombok.NonNull;
import org.littletonrobotics.junction.Logger;

public class CustomTrajectoryController {

  private static CustomTrajectoryController instance;
  private FullStateSwerveTrajectory trajectory;

  ErrorTracker errorTracker = new ErrorTracker(10);

  Timer timer = new Timer();

  PIDController x, y, r;

  CustomHolonomicDriveController controller;

  boolean doPrint = false;

  private CustomTrajectoryController() {
    controller = CustomHolonomicDriveController.getInstance();
  }

  public static CustomTrajectoryController getInstance() {
    if (instance == null) {
      instance = new CustomTrajectoryController();
    }
    return instance;
  }

  public void changePath(@NonNull FullStateSwerveTrajectory trajectory) {
    this.trajectory = trajectory;
    timer.stop();
    timer.reset();
    doPrint = true;
    errorTracker.reset();
  }

  public boolean isFinished() {
    return timer.hasElapsed(trajectory.getDuration());
  }

  public ChassisSpeeds update() {
    if (trajectory == null) {
      return new ChassisSpeeds();
    }

    if (timer.get() == 0) {
      timer.start();
    }

    var sample = trajectory.sample(timer.get());
    var z = controller.calculate(Robot.swerveDrive.getUsablePose(), sample);
    var error =
        new Pose2d(
            sample.getPose().minus(Robot.swerveDrive.getUsablePose()).getX(),
            sample.getPose().minus(Robot.swerveDrive.getUsablePose()).getY(),
            Robot.swerveDrive.getUsablePose().getRotation().minus(sample.getPose().getRotation()));

    errorTracker.addObservation(error);

    Logger.getInstance()
        .recordOutput(
            "Trajectory/Target Pose",
            new double[] {
              sample.getPose().getX(),
              sample.getPose().getY(),
              sample.getPose().getRotation().getDegrees()
            });
    Logger.getInstance().recordOutput("Trajectory/timer", timer.get());

    Logger.getInstance().recordOutput("Trajectory/Pose Error", error);

    if (isFinished()) {
      if (doPrint) {
        errorTracker.printSummary();
        doPrint = false;
      }
      return new ChassisSpeeds();

    } else {
      return z;
    }
  }
}
