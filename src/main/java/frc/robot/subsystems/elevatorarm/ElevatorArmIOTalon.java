/* (C) Robolancers 2025 */
package frc.robot.subsystems.elevatorarm;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.PWM1Configs;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Voltage;

/**
 * Implementation of the ElevatorArmIO that controls a real ElevatorArm using a TalonFX Motor
 * Controller
 */
@Logged
public class ElevatorArmIOTalon implements ElevatorArmIO {

  // tuning config for the ElevatorArmIOReal
  public static final ElevatorArmConfig config = new ElevatorArmConfig(0, 0, 0, 0, 0);

  // the motor that is controlling the arm (using a TalonFX controller)
  private TalonFX armMotor = new TalonFX(ElevatorArmConstants.kElevatorArmId);

  // TODO: if elec doesn't use a CANdi, use this for encoder output
  // absolute encoder from 0 to 360

  // private DutyCycleEncoder encoder =
  //     new DutyCycleEncoder(
  //         ElevatorArmConstants.kAbsoluteEncoderPort,
  //         360,
  //         ElevatorArmConstants.kAbsoluteEncoderOffset.in(Degrees));

  // CANdi device representing encoder
  private CANdi encoderCandi = new CANdi(ElevatorArmConstants.kEncoderCANdiId);

  // request to control the arm motor using voltage
  private VoltageOut voltageRequest = new VoltageOut(0);

  public ElevatorArmIOTalon() {
    // setup arm motor
    armMotor
        .getConfigurator()
        .apply(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(ElevatorArmConstants.kCurrentLimit));
    armMotor
        .getConfigurator()
        .apply(
            new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(
                    ElevatorArmConstants.kInverted
                        ? InvertedValue.CounterClockwise_Positive
                        : InvertedValue.Clockwise_Positive));
    armMotor
        .getConfigurator()
        .apply(
            new FeedbackConfigs()
                .withSensorToMechanismRatio(ElevatorArmConstants.kElevatorArmGearing));

    // setup encoder
    encoderCandi
        .getConfigurator()
        .apply(
            new PWM1Configs()
                .withAbsoluteSensorOffset(ElevatorArmConstants.kAbsoluteEncoderOffset));
  }

  // update inputs from the arm motor
  public void updateInputs(ElevatorArmInputs inputs) {
    // TODO: see line 31
    // inputs.angle = Degrees.of(encoder.get());
    inputs.angle = encoderCandi.getPWM1Position().getValue();
    inputs.velocity = armMotor.getVelocity().getValue();
    inputs.current = armMotor.getTorqueCurrent().getValue();
  }

  // set voltage to the arm motor
  public void setVoltage(Voltage volts) {
    armMotor.setControl(voltageRequest.withOutput(volts));
  }
}
