/* (C) Robolancers 2025 */
package frc.robot.subsystems.climbtest;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;

public class ClimbTest extends SubsystemBase {
  private SparkMax climbMotor = new SparkMax(48, MotorType.kBrushless);

  private Servo lockServo = new Servo(0);

  public ClimbTest() {
    climbMotor.configure( // configures two spark motors
        new SparkMaxConfig()
            .inverted(false)
            .voltageCompensation(12)
            .smartCurrentLimit(40)
            .apply(new EncoderConfig().velocityConversionFactor(1).positionConversionFactor(1)),
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    unlockServo();
  }

  public void setMechanismVoltage(Voltage volts) {
    climbMotor.setVoltage(volts);
  }

  public Command setMechanismVoltage(Supplier<Voltage> volts) {
    return run(
        () -> {
          setMechanismVoltage(volts.get());
        });
  }

  public void lockServo() {
    lockServo.setAngle(90);
  }

  public void unlockServo() {
    lockServo.setAngle(180);
  }
}
