package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Robot;

public class SpinIntakeDrop4BarElevator extends ParallelCommandGroup {
  public SpinIntakeDrop4BarElevator(
      double elevatorHeight, double fourBarAngle, double intakeVelocityRPM) {
    addCommands(
        Robot.intake.cmdSetVelocityRPM(intakeVelocityRPM),
        Robot.elevator.cmdSetTargetHeightAndWait(elevatorHeight),
        Robot.fourBar.cmdSetAngleDegAndWait(fourBarAngle));
  }
}
