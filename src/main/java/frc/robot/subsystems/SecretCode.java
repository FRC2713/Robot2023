package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class SecretCode extends SubsystemBase {
  private int steps = 0;

  public void resetCode() {
    steps = 0;
  }

  public void upPressed() {
    if (steps == 0 || steps == 1) {
      steps++;
    } else {
      resetCode();
    }
  }

  public void downPressed() {
    if (steps == 2 || steps == 3) {
      steps++;
    } else {
      resetCode();
    }
  }

  public void leftPressed() {
    if (steps == 4 || steps == 6) {
      steps++;
    } else {
      resetCode();
    }
  }

  public void rightPressed() {
    if (steps == 5 || steps == 7) {
      steps++;
    } else {
      resetCode();
    }
  }

  public void bPressed() {
    if (steps == 8) {
      steps++;
    } else {
      resetCode();
    }
  }

  public void aPressed() {
    if (steps == 9) {
      steps++;
    } else {
      resetCode();
    }
  }

  public void enterPressed() {
    if (steps == 10) {
      steps++;
    } else {
      resetCode();
    }
  }

  public void checkCode() {
    if (steps == 11) {
      Robot.elevator.setTargetHeight(30);
      resetCode();
    }
  }

  public static class Commands {
    public static Command upPress() {
      return new InstantCommand(() -> Robot.code.upPressed());
    }

    public static Command reset() {
      return new InstantCommand(() -> Robot.code.resetCode());
    }

    public static Command downPress() {
      return new InstantCommand(() -> Robot.code.downPressed());
    }

    public static Command leftPress() {
      return new InstantCommand(() -> Robot.code.leftPressed());
    }

    public static Command rightPress() {
      return new InstantCommand(() -> Robot.code.rightPressed());
    }

    public static Command bPress() {
      return new InstantCommand(() -> Robot.code.bPressed());
    }

    public static Command aPress() {
      return new InstantCommand(() -> Robot.code.aPressed());
    }

    public static Command enterPress() {
      return new InstantCommand(() -> Robot.code.enterPressed());
    }

    public static Command execute() {
      return new InstantCommand(() -> Robot.code.checkCode());
    }
  }
}
