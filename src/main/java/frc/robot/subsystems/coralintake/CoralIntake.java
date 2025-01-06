package frc.robot.subsystems.coralintake;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.coralintake.io.CoralIntakeIO;
import frc.robot.subsystems.coralintake.io.CoralIntakeIOIdeal;
import frc.robot.subsystems.coralintake.io.CoralIntakeIOReal;
import frc.robot.subsystems.coralintake.io.CoralIntakeIOSim;

public class CoralIntake extends SubsystemBase {
    private CoralIntakeInputs inputs; 
    private CoralIntakeIO io; 

    private PIDController speedController; 
    private SimpleMotorFeedforward feedforward;

    public static final CoralIntake create() {
        return RobotBase.isReal() ? 
            new CoralIntake(new CoralIntakeIOReal(), CoralIntakeIOReal.config) : 
            new CoralIntake(new CoralIntakeIOSim(), CoralIntakeIOSim.config); 
    }

    public static final CoralIntake disable() {
        return new CoralIntake(new CoralIntakeIOIdeal(), CoralIntakeIOIdeal.config); 
    }

    public CoralIntake(CoralIntakeIO io, CoralIntakeConfig config) {
        this.io = io; 
        this.inputs = new CoralIntakeInputs(); 
        this.speedController = new PIDController(config.kP(), config.kI(), config.kD()); 
        this.feedforward = new SimpleMotorFeedforward(config.kS(), config.kV()); 
    }

    public Command runAt(AngularVelocity desired) {
        return run(() -> {
            double volts = speedController.calculate(inputs.velocity.in(RPM), desired.in(RPM)) + feedforward.calculate(desired.in(RPM));
            io.setVoltage(Volts.of(volts)); 
        });
    }

    public Command intake() {
        return runAt(CoralIntakeConstants.kCoralIntakeSpeed);
    }
}
