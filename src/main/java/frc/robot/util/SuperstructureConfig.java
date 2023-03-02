package frc.robot.util;

import lombok.Builder;
import lombok.Getter;

@Builder
public class SuperstructureConfig {
  @Getter private double fourBarPosition;
  @Getter private double elevatorPosition;
  @Getter private double topRPM;
  @Getter private double bottomRPM;
}
