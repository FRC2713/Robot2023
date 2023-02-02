package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Robot;

public class RaiseElevatorExtendFourBarStopIntake extends ParallelCommandGroup {
  public RaiseElevatorExtendFourBarStopIntake(double elevatorHeight, double fourBarAngle) {
    addCommands(
        Robot.elevator.cmdSetTargetHeightAndWait(Units.metersToInches(elevatorHeight)),
        Robot.fourBar.cmdSetAngleDegAndWait(Units.radiansToDegrees(fourBarAngle)),
        Robot.intake.cmdSetVelocityRPM(0));
  }
}
