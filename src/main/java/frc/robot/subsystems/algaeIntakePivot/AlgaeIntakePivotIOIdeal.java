/* (C) Robolancers 2025 */
package frc.robot.subsystems.algaeIntakePivot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Voltage;

@Logged
public class AlgaeIntakePivotIOIdeal implements AlgaeIntakePivotIO {
  // this class is for when the robot disables
  public static final AlgaeIntakePivotConfig config = new AlgaeIntakePivotConfig(0, 0, 0, 0);

  public void updateInputs(AlgaeIntakePivotInputs inputs) {
    inputs.pivotAngle = Degrees.of(0);
    inputs.pivotVelocity = DegreesPerSecond.of(0);
    inputs.pivotCurrent = Amps.zero();
  }

  public void setPivotVoltage(Voltage volts) {}
}
