package frc.robot.util;

import edu.wpi.first.math.controller.*;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import java.util.HashSet;
import java.util.Set;
import lombok.NonNull;

public class PIDFFGains {
  public TunableNumber kP, kI, kD, kS, kV, kG, tolerance;
  public static Set<String> names = new HashSet<>();

  private PIDFFGains(PIDFFGainsBuilder builder) {
    String key = builder.name + "/";
    kP = new TunableNumber(key + "kP", builder.kP);
    kI = new TunableNumber(key + "kI", builder.kI);
    kD = new TunableNumber(key + "kD", builder.kD);
    kS = new TunableNumber(key + "kS", builder.kS);
    kV = new TunableNumber(key + "kV", builder.kV);
    kG = new TunableNumber(key + "kG", builder.kG);
    tolerance = new TunableNumber(key + "tolerance", builder.tolerance);
  }

  public boolean hasChanged() {
    return kP.hasChanged()
        || kI.hasChanged()
        || kD.hasChanged()
        || kS.hasChanged()
        || kV.hasChanged()
        || kG.hasChanged()
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

  public boolean hasChanged(ArmFeedforward feedforward) {
    return kS.get() != feedforward.ks || kV.get() != feedforward.kv || kG.get() != feedforward.kg;
  }

  public boolean hasChanged(PIDController controller, SimpleMotorFeedforward feedforward) {
    return hasChanged(controller) || hasChanged(feedforward);
  }

  public boolean hasChanged(PIDController controller, ArmFeedforward feedforward) {
    return hasChanged(controller) || hasChanged(feedforward);
  }

  public PIDController createWpilibController() {
    return new PIDController(kP.get(), kI.get(), kD.get());
  }

  public SimpleMotorFeedforward createWpilibFeedforward() {
    return new SimpleMotorFeedforward(kS.get(), kV.get());
  }

  public ArmFeedforward createArmFeedforward() {
    return new ArmFeedforward(kS.get(), kG.get(), kV.get());
  }

  public ElevatorFeedforward createElevatorFeedforward() {
    return new ElevatorFeedforward(kS.get(), kG.get(), kV.get());
  }

  public ProfiledPIDController createProfiledPIDController(
      TrapezoidProfile.Constraints constraints) {
    return new ProfiledPIDController(kP.get(), kI.get(), kD.get(), constraints);
  }

  public static PIDFFGainsBuilder builder(@NonNull String name) throws IllegalArgumentException {
    if (PIDFFGains.names.contains(name)) {
      throw new IllegalArgumentException("PIDFFGains with name " + name + " already exists!");
    }
    PIDFFGains.names.add(name);
    return new PIDFFGainsBuilder(name);
  }

  public static class PIDFFGainsBuilder {
    private String name;
    private double kP = 0, kI = 0, kD = 0, kS = 0, kV = 0, kG = 0;

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

    public PIDFFGainsBuilder kG(double kG) {
      this.kG = kG;
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
