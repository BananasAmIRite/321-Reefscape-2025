/* (C) Robolancers 2025 */
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.algaeIntakePivot.AlgaeIntakePivot;
import frc.robot.subsystems.algaeIntakeRollers.AlgaeIntakeRollers;
import frc.robot.util.TunableConstant;

public class AlgaeSuperstructure {

  private AlgaeIntakePivot pivot;
  private AlgaeIntakeRollers rollers;

  public AlgaeSuperstructure(AlgaeIntakePivot pivot, AlgaeIntakeRollers rollers) {
    this.pivot = pivot;
    this.rollers = rollers;
  }

  /* Moves the entire elevator+arm superstructure to a desired state;
  this should be the go-to way of moving the superstructure,
    aside from the default subsystem commands */

  public Command goToSetpoint(AlgaeSetpoint setpoint) {
    return pivot.goToAngle(setpoint);
  }

  public Command intakeAlgae() {
    return goToSetpoint(AlgaeSetpoint.INTAKE).alongWith(this.rollers.intake());
  }

  public Command outtakeAlgae() {
    return this.rollers.outtake();
  }

  public Command prepareClimb() {
    return goToSetpoint(AlgaeSetpoint.CLIMB_PREP);
  }

  public Command climb() {
    return this.pivot.climb();
  }

  public Command tune() {
    TunableConstant kTargetAngle = new TunableConstant("/AlgaeSuperstructure/TargetAngle", 0);

    return pivot.goToAngle(() -> Degrees.of(kTargetAngle.get()));
  }

  public boolean atTargetState() {
    return pivot.atSetpoint();
  }

  public boolean hasAlgae() {
    return rollers.hasAlgae();
  }

  public enum AlgaeSetpoint {
    NEUTRAL(Degrees.of(0)),
    INTAKE(Degrees.of(135)),
    OUTTAKE(Degrees.of(100)),
    CLIMB_PREP(Degrees.of(180));

    private Angle algaeAngle; // the angle the arm should go to

    AlgaeSetpoint(Angle algaeAngle) {
      this.algaeAngle = algaeAngle;
    }

    public Angle getAlgaeAngle() {
      return algaeAngle;
    }
  }
}
