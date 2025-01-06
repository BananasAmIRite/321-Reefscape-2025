package frc.robot.subsystems.coralintake;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;

public class CoralIntakeConstants {
    public static final int kCoralIntakeId = 0; 

    public static final int kCoralSensorPort = 0; 

    public static final AngularVelocity kCoralIntakeSpeed = RPM.of(2000); 


    public static final double kIntakeGearing = 1; 
    public static final double kIntakeMOI = 1; 
}
