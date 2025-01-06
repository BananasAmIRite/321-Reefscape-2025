package frc.robot.subsystems.coralintake.io;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.subsystems.coralintake.CoralIntakeConfig;
import frc.robot.subsystems.coralintake.CoralIntakeConstants;
import frc.robot.subsystems.coralintake.CoralIntakeInputs;

public class CoralIntakeIOSim implements CoralIntakeIO {

    public static final CoralIntakeConfig config = new CoralIntakeConfig(0, 0, 0, 0, 0); 

    private FlywheelSim intakeMotor = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), CoralIntakeConstants.kIntakeMOI, CoralIntakeConstants.kIntakeGearing), 
        DCMotor.getNEO(1)
    ); 
    private DigitalInput coralSensor = new DigitalInput(CoralIntakeConstants.kCoralSensorPort); 

    @Override
    public void updateInputs(CoralIntakeInputs inputs) {
        inputs.velocity = RPM.of(intakeMotor.getAngularVelocityRPM()); 
        inputs.current = Amps.of(intakeMotor.getCurrentDrawAmps()); 
        inputs.hasCoral = coralSensor.get();
    }

    @Override
    public void setVoltage(Voltage volts) {
        intakeMotor.setInputVoltage(volts.in(Volts));
    }
}
