/* (C) Robolancers 2025 */
package frc.robot.subsystems.algaeIntakePivot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.RobotConstants;

@Logged
public class AlgaeIntakePivotIOSim implements AlgaeIntakePivotIO {

  public static final AlgaeIntakePivotConfig config =
      new AlgaeIntakePivotConfig(0.2, 0, 0.01, 1.483);

  private SingleJointedArmSim pivotSim;

  public AlgaeIntakePivotIOSim() {
    // configures a simulated arm with two pivot motors controlling one pivot point
    pivotSim =
        new SingleJointedArmSim(
            LinearSystemId.createSingleJointedArmSystem(
                DCMotor.getNEO(2),
                AlgaeIntakePivotConstants.kPivotMOI,
                AlgaeIntakePivotConstants.kPivotGearing),
            DCMotor.getNEO(2),
            AlgaeIntakePivotConstants.kPivotGearing,
            AlgaeIntakePivotConstants.kPivotLength.in(Meters),
            AlgaeIntakePivotConstants.kPivotMinAngle.in(Radians),
            AlgaeIntakePivotConstants.kPivotMaxAngle.in(Radians),
            true,
            AlgaeIntakePivotConstants.kPivotStartingAngle.in(Radians));
  }

  public void setPivotVoltage(Voltage volts) {
    pivotSim.setInputVoltage(volts.in(Volts));
  }

  public void updateInputs(AlgaeIntakePivotInputs inputs) {
    // gets info from sim to update inputs
    pivotSim.update(RobotConstants.kRobotLoopPeriod.in(Seconds));
    inputs.pivotAngle = Radians.of(pivotSim.getAngleRads());
    inputs.pivotVelocity = RadiansPerSecond.of(pivotSim.getVelocityRadPerSec());
    inputs.pivotCurrent = Amps.of(pivotSim.getCurrentDrawAmps());
  }
}
