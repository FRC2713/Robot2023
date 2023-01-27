package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Robot;

public class MechanismManager {
  private final Mechanism2d mech;
  private final MechanismRoot2d root;
  private final MechanismLigament2d m_elevator;
  private final MechanismLigament2d m_four;
  private final MechanismLigament2d m_front;
  // private final MechanismLigament2d m_four_two;

  public MechanismManager() {
    mech = new Mechanism2d(50, 50);

    // Root node
    root = mech.getRoot("climber", 2, 0);

    // Subsystems

    // Elevator
    m_elevator =
        root.append(
            new MechanismLigament2d(
                "Elevator",
                0,
                Constants.ElevatorConstants.ELEVATOR_ANGLE_DEGREES,
                30,
                new Color8Bit(Color.kGreen)));

    m_four =
        m_elevator.append(
            new MechanismLigament2d(
                "Four", 12.657, Robot.four.getCurrentDegs(), 6, new Color8Bit(Color.kYellow)));

    // m_four_two =
    // m_elevator.append(
    //         new MechanismLigament2d(
    //             "Four 2",
    //             12.657,
    //             Units.radiansToDegrees(Robot.four.getCurrentRads()),
    //             6,
    //             new Color8Bit(Color.kAliceBlue)));

    m_front =
        m_four.append(new MechanismLigament2d("Front", 6, -90, 6, new Color8Bit(Color.kBlue)));

    // Log to SmartDashboard
    SmartDashboard.putData("Mech2d", mech);
  }

  public void periodic() {
    m_four.setAngle(
        Units.radiansToDegrees(Robot.four.getCurrentRads())
            + ElevatorConstants.ELEVATOR_ANGLE_DEGREES);
    // m_four_two.setAngle(Units.radiansToDegrees(Robot.four.getCurrentRads()));
    m_elevator.setLength(Robot.ele.getCurrentHeight());
    m_front.setAngle(-90 - Robot.four.getCurrentDegs());
  }
}
