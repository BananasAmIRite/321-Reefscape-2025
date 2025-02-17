/* (C) Robolancers 2025 */
package frc.robot.subsystems.elevator;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

@Logged
// "Inputs" into the elevator class from encoders, motors, etc.
public class ElevatorInputs {
  public Distance height;
  public LinearVelocity velocity;
  public Current current;
  public boolean atSetpoint;
}
