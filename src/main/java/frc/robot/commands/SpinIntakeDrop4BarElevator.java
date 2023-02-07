package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.elevatorIO.Elevator;
import frc.robot.subsystems.fourBarIO.FourBar;
import frc.robot.subsystems.intakeIO.Intake;

public class SpinIntakeDrop4BarElevator extends ParallelCommandGroup {
  public SpinIntakeDrop4BarElevator(
      double elevatorHeight, double fourBarAngle, double intakeVelocityRPM) {
    addCommands(
        Intake.Commands.setWheelVelocityRPM(intakeVelocityRPM),
        Intake.Commands.setRollerVelocityRPM(intakeVelocityRPM),
        Elevator.Commands.setTargetHeightAndWait(elevatorHeight),
        FourBar.Commands.setAngleDegAndWait(fourBarAngle));
  }
}
