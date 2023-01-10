package frc.robot.subsystems.SwerveIO.module;

public enum SwerveModules {
  FRONT_LEFT("FrontLeft"),
  FRONT_RIGHT("FrontRight"),
  BACK_LEFT("BackLeft"),
  BACK_RIGHT("BackRight");

  private final String string;

  SwerveModules(String name) {
    string = name;
  }

  @Override
  public String toString() {
    return string;
  }
}
