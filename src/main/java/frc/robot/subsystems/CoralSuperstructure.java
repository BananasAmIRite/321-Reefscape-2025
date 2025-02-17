/* (C) Robolancers 2025 */
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.coralendeffector.CoralEndEffector;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevatorarm.ElevatorArm;
import frc.robot.util.TunableConstant;
import java.util.function.Supplier;

public class CoralSuperstructure {

  private Elevator elevator;
  private ElevatorArm arm;
  private CoralEndEffector endEffector;

  private CoralScorerSetpoint targetState = CoralScorerSetpoint.FEED_CORAL;

  public CoralSuperstructure(Elevator elevator, ElevatorArm arm, CoralEndEffector endEffector) {
    this.elevator = elevator;
    this.arm = arm;
    this.endEffector = endEffector;
  }

  /* Moves the entire elevator+arm superstructure to a desired state; this should be the go-to way
  of moving the superstructure, aside from the default subsystem commands */
  public Command goToSetpoint(Supplier<CoralScorerSetpoint> setpoint) {
    return Commands.runOnce(() -> targetState = setpoint.get())
        .andThen(
            goToSetpoint(
                () -> setpoint.get().getElevatorHeight(), () -> setpoint.get().getArmAngle()));
  }

  public Command goToSetpoint(Supplier<Distance> height, Supplier<Angle> angle) {
    return elevator.goToHeight(() -> height.get()).alongWith(arm.goToAngle(() -> angle.get()));
  }

  public void goToSetpoint(CoralScorerSetpoint setpoint) {
    goToSetpoint(setpoint.getElevatorHeight(), setpoint.getArmAngle());
  }

  public void goToSetpoint(Distance height, Angle ang) {
    elevator.goToHeight(height);
    arm.goToAngle(ang);
  }

  public boolean atTargetState() {
    return elevator.atSetpoint() && arm.atSetpoint();
  }

  public Command feedCoral() {
    return goToSetpoint(() -> CoralScorerSetpoint.FEED_CORAL).alongWith(endEffector.intakeCoral());
  }

  public Command outtakeCoral() {
    return endEffector.outtakeCoral();
  }

  public Command tune() {
    TunableConstant kTargetDistance = new TunableConstant("/CoralSuperstructure/TargetDistance", 0);
    TunableConstant kTargetAngle = new TunableConstant("/CoralSuperstructure/TargetAngle", 0);

    return elevator
        .goToHeight(() -> Meters.of(kTargetDistance.get()))
        .alongWith(arm.goToAngle(() -> Degrees.of(kTargetAngle.get())));
  }

  public boolean hasCoral() {
    return endEffector.hasCoral();
  }

  public enum CoralScorerSetpoint {
    // TODO: determine angles empirically
    NEUTRAL(ElevatorConstants.kElevatorStartingHeight, Degrees.of(-60)), // TODO: make
    FEED_CORAL(Inches.of(40.058), Degrees.of(-77.64500)),
    L1(Inches.of(45), Degrees.of(30)), // TODO: actually tune
    L2(Inches.of(55), Degrees.of(50.13600)),
    L3(Inches.of(65), Degrees.of(57.56300)),
    L4(Inches.of(85), Degrees.of(56.57500)),
    ALGAE_LOW(Inches.of(50), Degrees.of(20)), // TODO: actually tune
    ALGAE_HIGH(Inches.of(60), Degrees.of(20)); // TODO: actually tune

    private Distance elevatorHeight; // the height of the elevator to got
    private Angle armAngle; // the angle the arm should go to

    CoralScorerSetpoint(Distance elevatorHeight, Angle armAngle) {
      this.armAngle = armAngle;
      this.elevatorHeight = elevatorHeight;
    }

    public Distance getElevatorHeight() {
      return elevatorHeight;
    }

    public Angle getArmAngle() {
      return armAngle;
    }
  }
}
