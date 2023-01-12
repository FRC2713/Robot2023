package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;

public class MechanismManager {
  private final Mechanism2d mech;
  private final MechanismRoot2d root;
  private final MechanismLigament2d m_elevator;

  public MechanismManager() {
    mech = new Mechanism2d(50, 50);

    // Root node
    root = mech.getRoot("climber", 2, 0);

    // Subsystems

    // Elevator
    m_elevator =
        root.append(
            new MechanismLigament2d("elevator", 0, Constants.Elevator.ELEVATOR_ANGLE_DEGREES));

    // Log to SmartDashboard
    SmartDashboard.putData("Mech2d", mech);
  }

  public void periodic() {
    m_elevator.setLength(Robot.ele.getCurrentHeight());
  }
}
