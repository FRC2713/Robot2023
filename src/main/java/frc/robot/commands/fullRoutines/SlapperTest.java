package frc.robot.commands.fullRoutines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class SlapperTest extends SequentialCommandGroup {
  public SlapperTest() {
    addCommands(
        // Slapper.Commands.sendItAndWait(), new WaitCommand(0.5), Slapper.Commands.comeBackHome()
        );
  }
}
