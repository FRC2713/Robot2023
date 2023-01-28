package frc.robot.subsystems.visionIO;

import frc.robot.subsystems.visionIO.Vision.SnapshotMode;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  @AutoLog
  public static class VisionInputs {
    public boolean validTarget;
    public double horizontalCrosshairOffset;
    public double verticalCrosshairOffset;
    public double targetArea;
    public double skew;
    public double pipelineLatency;
    public double shortSidelength;
    public double longSideLength;
    public double horizontalSideLength;
    public double verticalSideLength;
    public long pipelineIndex = 1;
    public double[] camtran;
    public long ID;
    public String jsonDump;
    public double[] botpose;
    public long neuralDetectorID;
    public long ledMode;
    public long camMode;
    public long pipeline;
    public long stream;
    public boolean snapshot; // IO layer
    public long crop;
  }

  public void updateInputs(VisionInputs inputs);

  public void setSnapshotMode(SnapshotMode mode);
}
