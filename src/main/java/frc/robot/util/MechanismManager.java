package frc.robot.util;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

public class MechanismManager {
  private final Mechanism2d mech;
  private final MechanismRoot2d root;
  private final MechanismRoot2d slapperRoot;
  private final MechanismLigament2d m_elevator;
  private final MechanismLigament2d m_slapper;
  private final MechanismLigament2d m_four;
  private final MechanismLigament2d m_front;
  // private final MechanismLigament2d m_four_two;

  public MechanismManager() {
    mech = new Mechanism2d(50, 50);
    slapperRoot = mech.getRoot("slapper", 15, 0);

    // Root node
    root = mech.getRoot("superstructure", 0, 0);

    // Subsystems

    // Slapper
    m_slapper =
        new MechanismLigament2d(
            "slapper_arm", Units.inchesToMeters(15), 0, 6, new Color8Bit(Color.kAzure));
    m_slapper.append(
        new MechanismLigament2d(
            "slapper_held_cone", Units.inchesToMeters(3), 0, 20, new Color8Bit(Color.kYellow)));

    // Elevator
    m_elevator =
        root.append(
            new MechanismLigament2d(
                "elevator",
                0,
                Constants.ElevatorConstants.ELEVATOR_ANGLE_DEGREES,
                30,
                new Color8Bit(Color.kGreen)));

    m_four =
        m_elevator.append(
            new MechanismLigament2d(
                "four_bar",
                12.657,
                Robot.fourBar.getCurrentDegs(),
                6,
                new Color8Bit(Color.kYellow)));

    m_front =
        m_four.append(
            new MechanismLigament2d("fourbar_front", 6, -90, 6, new Color8Bit(Color.kBlue)));

    // Log to SmartDashboard
    SmartDashboard.putData("Mech2d", mech);
  }

  public void periodic() {
    double elevatorHeightMeters = Units.inchesToMeters(Robot.elevator.getCurrentHeight());
    double fourbarDegrees = Robot.fourBar.getCurrentDegs();

    m_four.setAngle(fourbarDegrees - ElevatorConstants.ELEVATOR_ANGLE_DEGREES);
    m_elevator.setLength(elevatorHeightMeters);
    m_front.setAngle(-45 - Robot.fourBar.getCurrentDegs());
    // m_slapper.setAngle(Robot.slapper.getPositionDeg());

    update3d(elevatorHeightMeters, fourbarDegrees);
  }

  private void update3d(double elevatorHeightMeters, double fourbarDegrees) {
    double stageStart = Units.inchesToMeters(25.8);
    double actTravel =
        RedHawkUtil.lerp(
            0,
            ElevatorConstants.ELEVATOR_MAX_HEIGHT_METERS - stageStart,
            stageStart,
            ElevatorConstants.ELEVATOR_MAX_HEIGHT_METERS,
            Math.max(stageStart, elevatorHeightMeters));
    Pose3d elevatorPose3d =
        new Pose3d(
            actTravel * Math.cos(Units.degreesToRadians(ElevatorConstants.ELEVATOR_ANGLE_DEGREES)),
            0.0,
            actTravel * Math.sin(Units.degreesToRadians(ElevatorConstants.ELEVATOR_ANGLE_DEGREES)),
            new Rotation3d());

    actTravel = Math.min(ElevatorConstants.ELEVATOR_MAX_HEIGHT_METERS, elevatorHeightMeters);
    Pose3d carriagePose3d =
        new Pose3d(
            actTravel * Math.cos(Units.degreesToRadians(ElevatorConstants.ELEVATOR_ANGLE_DEGREES)),
            0.0,
            actTravel * Math.sin(Units.degreesToRadians(ElevatorConstants.ELEVATOR_ANGLE_DEGREES)),
            new Rotation3d());

    Pose3d fourbarTopArmPose3d =
        new Pose3d(
            carriagePose3d.getX() + 0.19,
            carriagePose3d.getY() + 0.0,
            carriagePose3d.getZ() + 0.33,
            new Rotation3d(0.0, Units.degreesToRadians(-fourbarDegrees), 0.0));

    Pose3d fourbarBotArmPose3d =
        new Pose3d(
            carriagePose3d.getX() + 0.305,
            carriagePose3d.getY() + 0.0,
            carriagePose3d.getZ() + 0.235,
            new Rotation3d(0.0, Units.degreesToRadians(-fourbarDegrees), 0.0));

    double fourbarArmLen = Units.inchesToMeters(10);
    Pose3d endEffectorPose3d =
        new Pose3d(
            fourbarTopArmPose3d.getX()
                + (fourbarArmLen * Math.cos(Units.degreesToRadians(fourbarDegrees))),
            fourbarTopArmPose3d.getY() + 0.0,
            fourbarTopArmPose3d.getZ()
                + (fourbarArmLen * Math.sin(Units.degreesToRadians(fourbarDegrees))),
            new Rotation3d());

    Logger.getInstance()
        .recordOutput(
            "Mechanism3d",
            elevatorPose3d,
            carriagePose3d,
            fourbarTopArmPose3d,
            fourbarBotArmPose3d,
            endEffectorPose3d);
  }
}
