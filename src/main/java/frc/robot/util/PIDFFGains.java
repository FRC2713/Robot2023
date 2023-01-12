package frc.robot.util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class PIDFFGains {
  public TunableNumber kP, kI, kD, kS, kV, tolerance;

  private PIDFFGains(PIDFFGainsBuilder builder) {
    String key = builder.name + "/";
    kP = new TunableNumber(key + "kP", builder.kP);
    kI = new TunableNumber(key + "kI", builder.kI);
    kD = new TunableNumber(key + "kD", builder.kD);
    kS = new TunableNumber(key + "kS", builder.kS);
    kV = new TunableNumber(key + "kV", builder.kV);
    tolerance = new TunableNumber(key + "tolerance", builder.tolerance);
  }

  public boolean hasChanged() {
    return kP.hasChanged()
        || kI.hasChanged()
        || kD.hasChanged()
        || kS.hasChanged()
        || kV.hasChanged()
        || tolerance.hasChanged();
  }

  public boolean hasChanged(PIDController controller) {
    return kP.get() != controller.getP()
        || kI.get() != controller.getI()
        || kD.get() != controller.getD()
        || tolerance.hasChanged();
  }

  public boolean hasChanged(SimpleMotorFeedforward feedforward) {
    return kS.get() != feedforward.ks || kV.get() != feedforward.kv;
  }

  public boolean hasChanged(PIDController controller, SimpleMotorFeedforward feedforward) {
    return hasChanged(controller) || hasChanged(feedforward);
  }

  public PIDController createWpilibController() {
    return new PIDController(kP.get(), kI.get(), kD.get());
  }

  public SimpleMotorFeedforward createWpilibFeedforward() {
    return new SimpleMotorFeedforward(kS.get(), kV.get());
  }

  public static PIDFFGainsBuilder builder(String name) {
    return new PIDFFGainsBuilder(name);
  }

  public static class PIDFFGainsBuilder {
    private String name;
    private double kP = 0, kI = 0, kD = 0, kS = 0, kV = 0;

    // we wind up overwriting wpilib's default tolerance, which is 0.05, so set the same default
    // here to keep the same functionality
    private double tolerance = 0.05;

    public PIDFFGainsBuilder(String name) {
      this.name = name;
    }

    public PIDFFGainsBuilder kP(double kP) {
      this.kP = kP;
      return this;
    }

    public PIDFFGainsBuilder kI(double kI) {
      this.kI = kI;
      return this;
    }

    public PIDFFGainsBuilder kD(double kD) {
      this.kD = kD;
      return this;
    }

    public PIDFFGainsBuilder kS(double kS) {
      this.kS = kS;
      return this;
    }

    public PIDFFGainsBuilder kV(double kV) {
      this.kV = kV;
      return this;
    }

    public PIDFFGainsBuilder tolerance(double tolerance) {
      this.tolerance = tolerance;
      return this;
    }

    public PIDFFGains build() {
      return new PIDFFGains(this);
    }
  }
}
