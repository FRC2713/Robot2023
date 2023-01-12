package frc.robot.subsystems.elevatorIO;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.NumericalIntegration;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class AngledElevatorSim extends ElevatorSim {

  private final boolean m_simulateGravity;

  // The min allowable height for the elevator.
  private final double m_minHeight;

  // The max allowable height for the elevator.
  private final double m_maxHeight;

  private Rotation2d m_angle;

  public AngledElevatorSim(
      DCMotor gearbox,
      double gearing,
      double carriageMassKg,
      double drumRadiusMeters,
      double minHeightMeters,
      double maxHeightMeters,
      boolean simulateGravity) {
    super(
        gearbox,
        gearing,
        carriageMassKg,
        drumRadiusMeters,
        minHeightMeters,
        maxHeightMeters,
        simulateGravity);
    this.m_simulateGravity = simulateGravity;
    this.m_maxHeight = maxHeightMeters;
    this.m_minHeight = minHeightMeters;
  }

  public AngledElevatorSim(
      DCMotor gearbox,
      double gearing,
      double carriageMassKg,
      double drumRadiusMeters,
      double minHeightMeters,
      double maxHeightMeters,
      boolean simulateGravity,
      Matrix<N1, N1> measurementStdDevs) {
    super(
        gearbox,
        gearing,
        carriageMassKg,
        drumRadiusMeters,
        minHeightMeters,
        maxHeightMeters,
        simulateGravity,
        measurementStdDevs);
    this.m_simulateGravity = simulateGravity;
    this.m_maxHeight = maxHeightMeters;
    this.m_minHeight = minHeightMeters;
  }

  public AngledElevatorSim(
      DCMotor gearbox,
      double gearing,
      double carriageMassKg,
      double drumRadiusMeters,
      double minHeightMeters,
      double maxHeightMeters,
      boolean simulateGravity,
      Matrix<N1, N1> measurementStdDevs,
      Rotation2d angle) {
    super(
        gearbox,
        gearing,
        carriageMassKg,
        drumRadiusMeters,
        minHeightMeters,
        maxHeightMeters,
        simulateGravity,
        measurementStdDevs);
    this.m_simulateGravity = simulateGravity;
    this.m_maxHeight = maxHeightMeters;
    this.m_minHeight = minHeightMeters;
    this.m_angle = angle;
  }

  public AngledElevatorSim(
      LinearSystem<N2, N1, N1> plant,
      DCMotor gearbox,
      double gearing,
      double drumRadiusMeters,
      double minHeightMeters,
      double maxHeightMeters,
      boolean simulateGravity) {
    super(
        plant,
        gearbox,
        gearing,
        drumRadiusMeters,
        minHeightMeters,
        maxHeightMeters,
        simulateGravity);
    this.m_simulateGravity = simulateGravity;
    this.m_maxHeight = maxHeightMeters;
    this.m_minHeight = minHeightMeters;
  }

  public AngledElevatorSim(
      LinearSystem<N2, N1, N1> plant,
      DCMotor gearbox,
      double gearing,
      double drumRadiusMeters,
      double minHeightMeters,
      double maxHeightMeters,
      boolean simulateGravity,
      Matrix<N1, N1> measurementStdDevs) {
    super(
        plant,
        gearbox,
        gearing,
        drumRadiusMeters,
        minHeightMeters,
        maxHeightMeters,
        simulateGravity,
        measurementStdDevs);
    this.m_simulateGravity = simulateGravity;
    this.m_maxHeight = maxHeightMeters;
    this.m_minHeight = minHeightMeters;
  }

  @Override
  protected Matrix<N2, N1> updateX(Matrix<N2, N1> currentXhat, Matrix<N1, N1> u, double dtSeconds) {
    // Calculate updated x-hat from Runge-Kutta.
    var updatedXhat =
        NumericalIntegration.rkdp(
            (x, _u) -> {
              Matrix<N2, N1> xdot = m_plant.getA().times(x).plus(m_plant.getB().times(_u));
              if (m_simulateGravity) {
                xdot =
                    xdot.plus(
                        VecBuilder.fill(
                            0, this.m_angle == null ? -9.8 : this.m_angle.getSin() * -9.8));
              }
              return xdot;
            },
            currentXhat,
            u,
            dtSeconds);

    // We check for collisions after updating x-hat.
    if (wouldHitLowerLimit(updatedXhat.get(0, 0))) {
      return VecBuilder.fill(m_minHeight, 0);
    }
    if (wouldHitUpperLimit(updatedXhat.get(0, 0))) {
      return VecBuilder.fill(m_maxHeight, 0);
    }
    return updatedXhat;
  }
}
