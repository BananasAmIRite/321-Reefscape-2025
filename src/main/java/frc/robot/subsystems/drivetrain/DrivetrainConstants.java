/* (C) Robolancers 2025 */
package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.RobotBase;

@Logged
public class DrivetrainConstants {
  public record AutoGains(double kP, double kI, double kD) {}

  public static final AutoGains kTranslationGains =
      RobotBase.isReal()
          ? new AutoGains(5, 0, 0) // real
          : new AutoGains(4, 0, 0.2); // sim

  public static final AutoGains kHeadingGains =
      RobotBase.isReal()
          ? new AutoGains(5, 0, 0) // real
          : new AutoGains(2, 0, 0.1); // sim

  public static final AutoGains tuneTranslationGains = new AutoGains(0, 0, 0); // isn't used
  public static final AutoGains tuneHeadingGains = new AutoGains(6, 0, 0); // for heading controller

  public static final Distance kTrackWidth = Inches.of(29);
  public static final Distance kWheelBase = Inches.of(29);

  public static final double kDriveDeadband = 0.05;
  public static final double kRotationDeadband = 0.05;
  public static final AngularVelocity kMaxAngularVelocity = RadiansPerSecond.of(Math.PI * 6);
  public static final LinearVelocity kMaxLinearVelocity =
      MetersPerSecond.of(4.0); // TunerConstants.kSpeedAt12Volts

  public static final LinearVelocity kSlowModeLinearVelocity = MetersPerSecond.of(2);
  public static final AngularVelocity kSlowModeAngularVelocity = RadiansPerSecond.of(Math.PI * 1);

  public static final Distance kAlignmentSetpointTranslationTolerance = Centimeters.one();
  public static final Angle kAlignmentSetpointRotationTolerance = Degrees.of(1.5);
}
