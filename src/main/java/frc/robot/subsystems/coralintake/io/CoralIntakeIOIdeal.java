package frc.robot.subsystems.coralintake.io;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.coralintake.CoralIntakeConfig;
import frc.robot.subsystems.coralintake.CoralIntakeInputs;

public class CoralIntakeIOIdeal implements CoralIntakeIO {

    public static final CoralIntakeConfig config = new CoralIntakeConfig(0, 0, 0, 0, 0); 
    
    @Override
    public void updateInputs(CoralIntakeInputs inputs) {
        inputs.velocity = RPM.of(0); 
        inputs.current = Amps.of(0); 
        inputs.hasCoral = false; 
    }

    @Override
    public void setVoltage(Voltage volts) {

    }
}
