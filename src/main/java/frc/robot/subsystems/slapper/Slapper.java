package frc.robot.subsystems.slapper;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Slapper extends SubsystemBase {
    private CANSparkMax slapperMotor;
    private int canID = 100;
    private int targetAngleDegrees = 0;
    public boolean isAtTarget;

    public Slapper() {
        // deviceID, MotorType
        slapperMotor = new CANSparkMax(canID, MotorType.kBrushless);
    }

    public void goToAngle(int angle) {
        targetAngleDegrees = angle;
    }

    // Runs every 20 ms
    public void periodic() {
        // Encoder Position = 0
        double pos = slapperMotor.getEncoder().getPosition();
        // 0 * 360 = 0
        pos *= 360;
        // 0 - 90 = -90
        double voltage = pos - targetAngleDegrees;
        // -90 / 10 = -9
        voltage *= Constants.SlapperConstants.DEMO_GAINS;
        // Send it!
        slapperMotor.setVoltage(voltage);

        isAtTarget = Math.abs(pos - targetAngleDegrees) < 0.1;
    }
}
