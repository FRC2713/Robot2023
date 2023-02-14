package frc.robot.commands.fullRoutines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.elevatorIO.Elevator;
import frc.robot.subsystems.fourBarIO.FourBar;
import frc.robot.subsystems.intakeIO.Intake;

public class TwoCargoUnder extends SequentialCommandGroup {
  public TwoCargoUnder() {
    addCommands(
        Elevator.Commands.elevatorConeHighScoreAndWait(),
        FourBar.Commands.extend(),
        Intake.Commands.setRollerVelocityRPM(100),
        Intake.Commands.setWheelVelocityRPM(100),
        new WaitCommand(1),
        FourBar.Commands.retract(),
        Intake.Commands.setRollerVelocityRPM(100),
        Intake.Commands.setWheelVelocityRPM(100),
        Elevator.Commands.setTargetHeightAndWait(Constants.zero));
  }
}
