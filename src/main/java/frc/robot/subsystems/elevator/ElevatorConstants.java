/* (C) Robolancers 2025 */
package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Pound;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Voltage;

public class ElevatorConstants {

  // Elevator IDs
  public static final int kLeftMotorID = 12;
  public static final int kRightMotorID = 0;

  // Elevator Physical Constants
  public static final double kElevatorGearing = 20;
  public static final Distance kElevatorConversion = Inches.of(0.375 * 2 * 13);

  public static final Mass kElevatorCarriageMass = Pound.of(20);
  public static final Distance kElevatorDrumRadius = kElevatorConversion.div(2 * Math.PI);
  public static final Distance kElevatorMinimumHeight = Inches.of(27);
  public static final Distance kElevatorMaximumHeight = Inches.of(56);
  public static final Distance kElevatorStartingHeight = kElevatorMinimumHeight;

  // Elevator Motor Configs
  public static final boolean kLeftInverted = false;
  public static final boolean kRightInverted = false;
  public static final int kCurrentLimit = 40;
  public static final double kPositionConversionFactor =
      kElevatorConversion.in(Meters) / kElevatorGearing;
  public static final double kVelocityConversionFactor = kPositionConversionFactor / 60;

  // Field heights & Angles for L2->4
  public static final Angle kLevel2Angle = Degrees.of(50.136);
  public static final Angle kLevel3Angle = Degrees.of(57.563);
  public static final Angle kLevel4Angle = Degrees.of(56.575);
  public static final Angle kIntakeAngle = Degrees.of(77.645).times(-1);
  public static final Distance kLevel2Height = Inches.of(34.079);
  public static final Distance kLevel3Height = Inches.of(46.166);
  public static final Distance kLevel4Height = Inches.of(71.524);
  public static final Distance kIntakeHeight = Inches.of(40.058);

  // Constants for homing elevator
  public static final Voltage kHomingVoltage = Volts.of(-2);
  public static final Current kHomingCurrentThreshold = Amps.of(25);
  public static final LinearVelocity kHomingVelocityThreshold = MetersPerSecond.of(0.5);
}
