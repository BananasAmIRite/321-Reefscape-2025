/* (C) Robolancers 2025 */
package frc.robot.subsystems.elevator;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;

@Logged
public interface ElevatorIO {

  // update inputs from sensors
  default void updateInputs(ElevatorInputs inputs) {}

  // run the motor at a specific voltage
  default void setVoltage(Voltage volts) {}

  // set encoder position; doesn't run the motor
  default void resetEncoderPosition() {}

  // run elevator to position using onboard pid controller
  default void goToPosition(Distance position) {}

  // set pid values of onboard pid controller
  default void setOnboardPID(ElevatorConfig config) {}
}
