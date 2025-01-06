package frc.robot.subsystems.coralintake.io;

import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.coralintake.CoralIntakeInputs;

public interface CoralIntakeIO {
    default void updateInputs(CoralIntakeInputs inputs) {}
    default void setVoltage(Voltage volts) {} 
}
