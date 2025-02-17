/* (C) Robolancers 2025 */
package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;

@Logged
// For when Elevator is real
public class ElevatorIOSpark implements ElevatorIO {
  // Creates config record w/ values
  public static final ElevatorConfig config = new ElevatorConfig(0, 0, 0, 0, 0, 0, 0);
  // Creates motor objects
  public SparkMax elevatorMotorLeft =
      new SparkMax(ElevatorConstants.kLeftMotorID, MotorType.kBrushless);

  public SparkMax elevatorMotorRight =
      new SparkMax(ElevatorConstants.kRightMotorID, MotorType.kBrushless);

  private SparkClosedLoopController pidController = elevatorMotorRight.getClosedLoopController();

  private ElevatorFeedforward feedforward = new ElevatorFeedforward(0, 0, 0);

  private Distance lastReference = Meters.of(0);

  // Constructor: Sets up motors
  public ElevatorIOSpark() {
    setupMotors();
    setOnboardPID(config);
  }

  // Updates inputs with values from encoder (Called periodically in periodic function later)
  public void updateInputs(ElevatorInputs inputs) {
    inputs.height = Meters.of(elevatorMotorLeft.getEncoder().getPosition());
    inputs.velocity = MetersPerSecond.of(elevatorMotorLeft.getEncoder().getVelocity());
    inputs.current = Amps.of(elevatorMotorLeft.getOutputCurrent());
    inputs.atSetpoint =
        Math.abs(elevatorMotorLeft.getEncoder().getPosition() - lastReference.in(Meters))
            < ElevatorConstants.kHeightTolerance.in(Meters);
    ;
  }

  // Method to setup L & R motor & encoders
  // NOTE: Right motor follows left & only left motor encoder is used
  private void setupMotors() {
    elevatorMotorRight.configure(
        new SparkMaxConfig()
            .smartCurrentLimit((int) ElevatorConstants.kStatorLimit.in(Amps))
            .voltageCompensation(ElevatorConstants.kNominalVoltage.in(Volts))
            .idleMode(IdleMode.kBrake)
            .inverted(ElevatorConstants.kRightInverted)
            .apply(
                new EncoderConfig()
                    .velocityConversionFactor(ElevatorConstants.kVelocityConversionFactor)
                    .positionConversionFactor(ElevatorConstants.kPositionConversionFactor)),
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    elevatorMotorLeft.configure(
        new SparkMaxConfig()
            .smartCurrentLimit((int) ElevatorConstants.kStatorLimit.in(Amps))
            .voltageCompensation(ElevatorConstants.kNominalVoltage.in(Volts))
            .idleMode(IdleMode.kBrake)
            .apply(
                new EncoderConfig()
                    .velocityConversionFactor(ElevatorConstants.kVelocityConversionFactor)
                    .positionConversionFactor(ElevatorConstants.kPositionConversionFactor))
            .follow(elevatorMotorRight, ElevatorConstants.kLeftInverted),
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  // Sets power of motors w/voltage
  public void setVoltage(Voltage Volts) {
    elevatorMotorRight.setVoltage(Volts);
  }

  // resets encoder pos
  @Override
  public void resetEncoderPosition() {
    elevatorMotorLeft
        .getEncoder()
        .setPosition(ElevatorConstants.kElevatorStartingHeight.in(Meters));
    elevatorMotorRight
        .getEncoder()
        .setPosition(ElevatorConstants.kElevatorStartingHeight.in(Meters));
  }

  @Override
  public void goToPosition(Distance position) {
    double ffOutput = feedforward.calculate(0);
    lastReference = position;
    pidController.setReference(
        position.in(Meters), ControlType.kPosition, ClosedLoopSlot.kSlot0, ffOutput);
  }

  @Override
  public void setOnboardPID(ElevatorConfig config) {
    elevatorMotorRight.configure(
        new SparkMaxConfig()
            .apply(new ClosedLoopConfig().p(config.kP()).i(config.kI()).d(config.kD())),
        ResetMode.kNoResetSafeParameters,
        PersistMode.kPersistParameters);
    feedforward = new ElevatorFeedforward(config.kS(), config.kG(), config.kV(), config.kA());
  }
}
