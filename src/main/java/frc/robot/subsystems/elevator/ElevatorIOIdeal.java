/* (C) Robolancers 2025 */
package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;

@Logged
// Ideal case for use when Elevator is disabled, doesn't actually control anything
public class ElevatorIOIdeal implements ElevatorIO {

  public static ElevatorConfig config = new ElevatorConfig(0, 0, 0, 0, 0, 0, 0);

  /* updateInputs method updates the Inputs with the actual values from motors;
  Called repeatedly in periodic method in Elevator but doesn't do anything here */
  public void updateInputs(ElevatorInputs inputs) {
    inputs.height = ElevatorConstants.kElevatorStartingHeight;
    inputs.velocity = MetersPerSecond.of(0);
    inputs.current = Amps.of(0);
  }

  /* Sets power to motors in Volts (Both motors bc Right motor has not been setup to follow
  left motor) */
  public void setVoltage(Voltage Volts) {}

  // Sets encoder position of left motor
  public void setEncoderPosition(Distance height) {}
}
