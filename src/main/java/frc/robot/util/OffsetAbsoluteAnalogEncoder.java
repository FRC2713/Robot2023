package frc.robot.util;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AnalogEncoder;

public class OffsetAbsoluteAnalogEncoder {

  private final double voltageOffset;
  private AnalogEncoder analogEncoder;

  private LinearFilter offsetStabilizer = LinearFilter.movingAverage(20);

  public OffsetAbsoluteAnalogEncoder(int port, double voltageOffset) {
    this.analogEncoder = new AnalogEncoder(port);
    this.voltageOffset = voltageOffset;
  }

  public OffsetAbsoluteAnalogEncoder(int port) {
    this(port, 0.0);
  }

  public double getUnadjustedVoltage() {
    return analogEncoder.getAbsolutePosition();
  }

  public double getAdjustedVoltage() {
    return analogEncoder.getAbsolutePosition() - voltageOffset;
  }

  public double getFilteredOffset() {
    return offsetStabilizer.calculate(analogEncoder.getAbsolutePosition());
  }

  public Rotation2d getUnadjustedRotation2d() {
    return Rotation2d.fromDegrees(getUnadjustedVoltage() * 360.0);
  }

  public Rotation2d getAdjustedUnconstrainedRotation2d() {
    return Rotation2d.fromDegrees(getAdjustedVoltage() * 360.0);
  }

  public Rotation2d getAdjustedRotation2d() {
    return RedHawkUtil.constrainToRange(
        getAdjustedUnconstrainedRotation2d(),
        Rotation2d.fromDegrees(-180),
        Rotation2d.fromDegrees(180));
  }
}
