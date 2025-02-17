/* (C) Robolancers 2025 */
package frc.robot.subsystems.elevatorarm;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;

import com.ctre.phoenix6.configs.PWM1Configs;
import com.ctre.phoenix6.hardware.CANdi;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Voltage;

/**
 * Implementation of the ElevatorArmIO that controls a real ElevatorArm using a SparkMax motor
 * controller
 */
@Logged
public class ElevatorArmIOSpark implements ElevatorArmIO {

  // tuning config for the ElevatorArmIOReal
  public static final ElevatorArmConfig config = new ElevatorArmConfig(0, 0, 0, 0, 0);

  // the motor that is controlling the arm (using a SparkMax controller)
  private SparkMax armMotor =
      new SparkMax(ElevatorArmConstants.kElevatorArmId, MotorType.kBrushless);

  // TODO: if elec uses direct wiring, use this for encoder output
  // absolute encoder from 0 to 360
  // private DutyCycleEncoder armEncoder =
  //     new DutyCycleEncoder(
  //         ElevatorArmConstants.kAbsoluteEncoderPort,
  //         360,
  //         ElevatorArmConstants.kAbsoluteEncoderOffset.in(Degrees));

  private CANdi encoderCandi = new CANdi(ElevatorArmConstants.kEncoderCANdiId);

  public ElevatorArmIOSpark() {
    // setup arm motor
    armMotor.configure(
        new SparkMaxConfig() // config for basic motor stuff
            .smartCurrentLimit(ElevatorArmConstants.kCurrentLimit)
            .voltageCompensation(ElevatorArmConstants.kNominalVoltage)
            .idleMode(IdleMode.kBrake)
            .inverted(ElevatorArmConstants.kInverted)
            .apply(
                new EncoderConfig() // config for relative encoder
                    .positionConversionFactor(ElevatorArmConstants.kPositionConversionFactor)
                    .velocityConversionFactor(ElevatorArmConstants.kVelocityConversionFactor)),
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // setup encoder
    encoderCandi
        .getConfigurator()
        .apply(
            new PWM1Configs()
                .withAbsoluteSensorOffset(ElevatorArmConstants.kAbsoluteEncoderOffset));
  }

  // update inputs from the arm motor
  public void updateInputs(ElevatorArmInputs inputs) {
    inputs.angle = encoderCandi.getPWM1Position().getValue();
    inputs.velocity = DegreesPerSecond.of(armMotor.getEncoder().getVelocity());
    inputs.current = Amps.of(armMotor.getOutputCurrent());
  }

  // set voltage to the arm motor
  public void setVoltage(Voltage volts) {
    armMotor.setVoltage(volts);
  }
}
