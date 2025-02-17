/* (C) Robolancers 2025 */
package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Pound;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Voltage;

public class ElevatorConstants {

  // Elevator IDs
  public static final int kLeftMotorID = 13;
  public static final int kRightMotorID = 14;

  // Elevator Physical Constants
  public static final double kElevatorGearing = 20;
  public static final Distance kElevatorConversion = Inches.of(0.375 * 2 * 13);

  public static final Mass kElevatorCarriageMass = Pound.of(20); // TODO: get carriage mass
  public static final Distance kElevatorDrumRadius = kElevatorConversion.div(2 * Math.PI);
  public static final Distance kElevatorMinimumHeight = Inches.of(35);
  public static final Distance kElevatorMaximumHeight = Inches.of(87.75);
  public static final Distance kElevatorStartingHeight = kElevatorMinimumHeight;

  public static final Distance kElevatorDangerHeight =
      Meters.of(0.9); // TODO: get this danger height

  // controller config
  public static final Distance kHeightTolerance = Meters.of(0.1);

  // Elevator Motor Configs
  public static final boolean kLeftInverted = false;
  public static final boolean kRightInverted = true;
  public static final boolean kFollowerInverted = true;
  public static final Current kStatorLimit = Amps.of(40);
  public static final Current kSupplyLimit = Amps.of(40);
  public static final double kPositionConversionFactor =
      kElevatorConversion.in(Meters) / kElevatorGearing;
  public static final double kVelocityConversionFactor = kPositionConversionFactor / 60;
  public static final LinearVelocity kMaxVelocity = MetersPerSecond.of(2);
  public static final LinearAcceleration kMaxAcceleration = MetersPerSecondPerSecond.of(10);

  // Constants for homing elevator
  public static final Voltage kHomingVoltage = Volts.of(-2);
  public static final Current kHomingCurrentThreshold = Amps.of(20);
  public static final LinearVelocity kHomingVelocityThreshold = MetersPerSecond.of(0.5);

  public static final Voltage kNominalVoltage = Volts.of(12);
}
