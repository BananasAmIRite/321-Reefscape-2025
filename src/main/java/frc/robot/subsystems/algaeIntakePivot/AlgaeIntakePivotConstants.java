/* (C) Robolancers 2025 */
package frc.robot.subsystems.algaeIntakePivot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;

@Logged
public class AlgaeIntakePivotConstants {
  // TODO: get all these constants

  // motor, encoder, sensor IDs
  public static final int kPivotMotorLeftId = 0;
  public static final int kPivotMotorRightId = 0;

  // pivot thresholds
  public static final Angle kMinBlockedAngle = Degrees.of(20); // TODO: find these from mechanical
  public static final Angle kMaxBlockedAngle = Degrees.of(40);
  // setpoint constants
  public static final Angle kPivotClimbThreshold = Degrees.of(45); // TODO: tune

  // pivot homing
  public static final Voltage kHomingVoltage = Volts.of(-2);
  public static final Current kHomingCurrentThreshold = Amps.of(25);
  public static final AngularVelocity kHomingVelocityThreshold = DegreesPerSecond.of(2);

  // physical constants
  public static final double kPivotGearing =
      22.0 / 14.0 * 64.0 / 16.0 * 25.0 / 1; // TODO: find these from mechanical
  public static final Distance kPivotLength = Inches.of(0.6); // TODO: tune
  public static final Angle kPivotMinAngle = Degrees.of(0);
  public static final Angle kPivotMaxAngle = Degrees.of(180);
  public static final Angle kPivotStartingAngle = Degrees.of(0);
  public static final double kPivotMOI = 0.1; // TODO: tune
  public static final Voltage kPivotClimbVoltage = Volts.of(-8); // TODO: tune

  // controller constants
  public static final Angle kControllerTolerance = Degrees.of(1);

  // motor configurations
  public static final boolean kLeftInverted = false;
  public static final boolean kRightInverted = false;
  public static final int kSmartCurrentLimit = 40;
  public static final double kPivotPositionConversionFactor = 360 / kPivotGearing;
  public static final double kPivotVelocityConversionFactor = kPivotPositionConversionFactor / 60;
  public static final Voltage kNominalVoltage = Volts.of(12);
}
