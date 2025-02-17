/* (C) Robolancers 2025 */
package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.RobotConstants;

@Logged
// Simulating the Elevator
public class ElevatorIOSim implements ElevatorIO {
  // Constant values for the simulation of the elevator
  public static final ElevatorConfig config = new ElevatorConfig(50, 0, 0, 0.404, 0, 0, 0);

  // controllers
  private PIDController pidController = new PIDController(0, 0, 0);
  private ElevatorFeedforward feedforward = new ElevatorFeedforward(0, 0, 0);

  /* Elevator sim object with appropriate paramaters
  Creates the motor simulation of the elevator */
  public ElevatorSim simMotor =
      new ElevatorSim(
          DCMotor.getNEO(2),
          ElevatorConstants.kElevatorGearing,
          ElevatorConstants.kElevatorCarriageMass.in(Kilogram),
          ElevatorConstants.kElevatorDrumRadius.in(Meters),
          ElevatorConstants.kElevatorMinimumHeight.in(Meters),
          ElevatorConstants.kElevatorMaximumHeight.in(Meters),
          true,
          ElevatorConstants.kElevatorStartingHeight.in(Meters));

  public ElevatorIOSim() {
    setOnboardPID(config);
  }

  // Updates Inputs w/ values from sim (Also tells sim how often to update itself)
  public void updateInputs(ElevatorInputs inputs) {
    simMotor.update(RobotConstants.kRobotLoopPeriod.in(Seconds));
    inputs.height = Meters.of(simMotor.getPositionMeters());
    inputs.velocity = MetersPerSecond.of(simMotor.getVelocityMetersPerSecond());
    inputs.current = Amp.of(simMotor.getCurrentDrawAmps());
    inputs.atSetpoint = pidController.atSetpoint();
  }

  // Method that sets power of the motors w/volts
  public void setVoltage(Voltage volts) {
    simMotor.setInputVoltage(volts.in(Volts));
  }

  // resets encoder position to its starting height
  @Override
  public void resetEncoderPosition() {
    simMotor.setState(
        ElevatorConstants.kElevatorStartingHeight.in(Meters),
        simMotor.getVelocityMetersPerSecond());
  }

  @Override
  public void goToPosition(Distance dist) {
    // TODO: referencing motor position specifically here is iffy, find a way to refactor
    double motorOutput = pidController.calculate(simMotor.getPositionMeters(), dist.in(Meters));

    double ff = feedforward.calculate(motorOutput);

    setVoltage(Volts.of(motorOutput + ff));
  }

  @Override
  public void setOnboardPID(ElevatorConfig config) {
    this.pidController.setPID(config.kP(), config.kI(), config.kD());
    this.feedforward = new ElevatorFeedforward(config.kS(), config.kG(), config.kV(), config.kA());
  }
}
