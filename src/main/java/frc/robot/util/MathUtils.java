/* (C) Robolancers 2025 */
package frc.robot.util;

public class MathUtils {

  // TODO: no wpilib alternative but remove if left unused
  public static double squareKeepSign(double x) {
    return Math.signum(x) * x * x;
  }
}
