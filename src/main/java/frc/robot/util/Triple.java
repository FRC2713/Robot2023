package frc.robot.util;

import lombok.Getter;

public class Triple<A, B, C> {
  @Getter private final A first;
  @Getter private final B second;
  @Getter private final C third;

  public Triple(A a, B b, C c) {
    first = a;
    second = b;
    third = c;
  }
}
