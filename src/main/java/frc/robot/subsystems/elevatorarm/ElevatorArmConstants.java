/* (C) Robolancers 2025 */
package frc.robot.subsystems.elevatorarm;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class ElevatorArmConstants {
  // the CAN ID of the arm motor on the elevator
  public static final int kElevatorArmId = 0;
  public static final int kEncoderCANdiId = 0;

  // the gearing of the arm
  public static final double kElevatorArmGearing = 60.45;
  // conversion factors for the relative encoder into angle in degrees
  public static final double kPositionConversionFactor = 360 / kElevatorArmGearing;
  public static final double kVelocityConversionFactor = kPositionConversionFactor / 60;

  // motor characteristics
  public static final int kCurrentLimit = 40;
  public static final double kNominalVoltage = 12;
  public static final boolean kInverted = false;

  // absolute encoder port on RoboRIO DIO if we need it
  public static final int kAbsoluteEncoderPort = 0;

  // absolute encoder offset
  public static final Angle kAbsoluteEncoderOffset = Degrees.of(0);

  // non-setpoint setpoints
  public static final Angle kPreAlignAngle = Degrees.of(150);

  // controller config
  public static final Angle kAngleTolerance = Degrees.of(1);

  // **** For simulation use ****
  // the MOI of the arm
  public static final double kElevatorArmMOI = 1; // TODO: find

  // the effective length of the arm
  public static final Distance kElevatorArmLength = Meters.of(0.5); // TODO: find

  // the length of the arm (shoulder to elbow)
  public static final Distance kArmLength = Meters.of(0.5); // TODO: find

  // the length of the arm (elbow to wrist)
  public static final Distance kElbowLength = Inches.of(10.98); // TODO: find

  // the fixed angle of the elbow (shoulder to elbow to wrist angle)
  public static final Angle kElbowAngle = Degrees.of(80); // TODO: find

  // the maximum angle the arm can go in simulation
  public static final Angle kMaxAngle = Degrees.of(180);

  // the minimum angle the arm can go in simulation
  public static final Angle kMinAngle = Degrees.of(-180);

  // the angle from the encoder rotation axis to the center of mass
  public static final Angle kCMOffset = Degrees.of(-10); // TODO: find

  // the starting angle of the arm
  public static final Angle kStartAngle = Degrees.of(-64.53);
}
