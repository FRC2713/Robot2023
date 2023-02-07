package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.elevatorIO.Elevator;
import frc.robot.subsystems.fourBarIO.FourBar;
import frc.robot.subsystems.intakeIO.Intake;

public class RaiseElevatorExtendFourBarStopIntake extends ParallelCommandGroup {
  public RaiseElevatorExtendFourBarStopIntake(double elevatorHeight, double fourBarAngle) {
    addCommands(
        Elevator.Commands.setTargetHeightAndWait(Units.metersToInches(elevatorHeight)),
        FourBar.Commands.setAngleDegAndWait(Units.radiansToDegrees(fourBarAngle)),
        Intake.Commands.setWheelVelocityRPM(0),
        Intake.Commands.setRollerVelocityRPM(0));
  }
}
