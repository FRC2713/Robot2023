package frc.robot.subsystems.lightStripIO;

public interface LightStripIO {

  public static class LightStripInputs {
    public double patternValue = -0.99;
  }

  public void updateInputs(LightStripInputs inputs);

  public void setPattern();
}
