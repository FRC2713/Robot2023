package frc.robot.util;

import static frc.robot.Constants.DriveConstants.Gains;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
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
          Gains.K_TRAJECTORY_CONTROLLER_GAINS_X.createWpilibController(),
          Gains.K_TRAJECTORY_CONTROLLER_GAINS_Y.createWpilibController(),
          Gains.K_TRAJECTORY_CONTROLLER_GAINS_ROTATION.createWpilibController());

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
    Logger.getInstance().recordOutput("Trajectory/Total time", newTrajectory.getTotalTimeSeconds());
    Logger.getInstance()
        .recordOutput(
            "Trajectory/Initial State Velocity",
            newTrajectory.getInitialState().velocityMetersPerSecond);

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

    if (isFinished()) {
      targetState = traj.getEndState();
    } else {
      targetState = (PathPlannerState) traj.sample(timer.get());
    }

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
      final var loopTime = 0.02;
      var speeds = controller.calculate(Robot.swerveDrive.getUsablePose(), targetState);

      Pose2d robotPoseVel =
          new Pose2d(
              speeds.vxMetersPerSecond * loopTime,
              speeds.vyMetersPerSecond * loopTime,
              Rotation2d.fromRadians(speeds.omegaRadiansPerSecond * loopTime));
      Twist2d twistVel = RedHawkUtil.poseLog(robotPoseVel);
      ChassisSpeeds updatedSpeeds =
          new ChassisSpeeds(
              twistVel.dx / loopTime, twistVel.dy / loopTime, twistVel.dtheta / loopTime);
      return updatedSpeeds;
    } else return new ChassisSpeeds();
  }
}
