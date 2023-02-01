package frc.robot.util;

import static frc.robot.Constants.DriveConstants.Gains;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import java.util.HashMap;
import lombok.NonNull;
import org.littletonrobotics.junction.Logger;

public class TrajectoryController {

  private static TrajectoryController instance;
  Timer timer = new Timer();
  PathPlannerTrajectory traj;
  HashMap<String, Command> eventMap = new HashMap<>();
  PathPlannerState targetState;
  PPHolonomicDriveController controller =
      new PPHolonomicDriveController(
          new PIDFFController(Gains.kTrajectoryControllerGainsX),
          new PIDFFController(Gains.kTrajectoryControllerGainsY),
          new PIDFFController(Gains.kTrajectoryControllerGainsRotation));

  private TrajectoryController() {}

  public static TrajectoryController getInstance() {
    if (instance == null) {
      instance = new TrajectoryController();
    }
    return instance;
  }

  public void changePath(@NonNull PathPlannerTrajectory newTrajectory) {
    traj = newTrajectory;

    Logger.getInstance().recordOutput("Trajectory/Trajectory Obj", newTrajectory);
    timer.reset();
    timer.stop();
  }

  public boolean isFinished() {
    return timer.get() >= traj.getTotalTimeSeconds();
  }

  public ChassisSpeeds update() {
    if (traj == null) {
      return new ChassisSpeeds();
    }
    if (timer.get() == 0) {
      timer.start();
    }

    targetState = (PathPlannerState) traj.sample(timer.get());

    Logger.getInstance()
        .recordOutput(
            "Trajectory/Target Pose",
            new double[] {
              targetState.poseMeters.getX(),
              targetState.poseMeters.getY(),
              targetState.holonomicRotation.getDegrees()
            });
    Logger.getInstance().recordOutput("Trajectory/timer", timer.get());
    if (!isFinished()) {
      return controller.calculate(Robot.swerveDrive.getRegularPose(), targetState);
    } else return new ChassisSpeeds();
  }
}
