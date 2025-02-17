/* (C) Robolancers 2025 */
package frc.robot.subsystems.coralendeffector;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotConstants;

// Implementation of the CoralEndEffectorIO that controls the simulated coral end effector
@Logged
class CoralEndEffectorIOSim implements CoralEndEffectorIO {

  // Constant values for the simulation of the coral end effector
  public static final CoralEndEffectorConfig config =
      new CoralEndEffectorConfig(0.1, 0, 0, 0.00208);

  public DCMotorSim simulation =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getNEO(1),
              CoralEndEffectorConstants.momentOfInertia,
              CoralEndEffectorConstants.gearing),
          DCMotor.getNEO(1));

  public CoralEndEffectorIOSim() {
    SmartDashboard.putBoolean("/SimInputs/CoralEndEffector/HasCoral", false);
  }

  @Override
  public void updateInputs(CoralEndEffectorInputs inputs) {
    simulation.update(RobotConstants.kRobotLoopPeriod.in(Seconds));
    inputs.voltage = Volts.of(simulation.getInputVoltage());
    inputs.velocity = RPM.of(simulation.getAngularVelocityRPM());
    inputs.hasCoral = SmartDashboard.getBoolean("/SimInputs/CoralEndEffector/HasCoral", false);
  }

  @Override
  public void setVoltage(Voltage voltage) {
    simulation.setInputVoltage(voltage.in(Volts));
  }
}
