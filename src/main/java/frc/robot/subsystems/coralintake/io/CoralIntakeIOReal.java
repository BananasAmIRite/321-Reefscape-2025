package frc.robot.subsystems.coralintake.io;

import com.revrobotics.spark.SparkMax;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.subsystems.coralintake.CoralIntakeConfig;
import frc.robot.subsystems.coralintake.CoralIntakeConstants;
import frc.robot.subsystems.coralintake.CoralIntakeInputs;

public class CoralIntakeIOReal implements CoralIntakeIO {

    public static final CoralIntakeConfig config = new CoralIntakeConfig(0, 0, 0, 0, 0); 

    private SparkMax intakeMotor = new SparkMax(CoralIntakeConstants.kCoralIntakeId, MotorType.kBrushless); 
    private DigitalInput coralSensor = new DigitalInput(CoralIntakeConstants.kCoralSensorPort); 
    @Override
    public void updateInputs(CoralIntakeInputs inputs) {
        inputs.velocity = RPM.of(intakeMotor.getEncoder().getVelocity()); 
        inputs.current = Amps.of(intakeMotor.getOutputCurrent()); 
        inputs.hasCoral = coralSensor.get();
    }

    @Override
    public void setVoltage(Voltage volts) {
        intakeMotor.setVoltage(volts);
    }
}
