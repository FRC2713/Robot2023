package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Robot;
import frc.robot.subsystems.fourBarIO.FourBar;
import frc.robot.subsystems.intakeIO.Intake;

public class RaiseElevatorExtendFourBarStopIntake extends ParallelCommandGroup {
  public RaiseElevatorExtendFourBarStopIntake(double elevatorHeight, double fourBarAngle) {
    addCommands(
        Robot.elevator.cmdSetTargetHeightAndWait(Units.metersToInches(elevatorHeight)),
        FourBar.Commands.cmdSetAngleDegAndWait(Units.radiansToDegrees(fourBarAngle)),
        Intake.Commands.cmdSetWheelVelocityRPM(0),
        Intake.Commands.cmdSetRollerVelocityRPM(0));
  }
}
