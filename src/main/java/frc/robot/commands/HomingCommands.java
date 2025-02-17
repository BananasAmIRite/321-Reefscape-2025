/* (C) Robolancers 2025 */
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.algaeIntakePivot.AlgaeIntakePivot;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;

public class HomingCommands {
  /**
   * Routine to home all mechanisms on the robot (eg. algae pivot and the elevator) Homes the
   * elevator first, then moves elevator out of the way and homes the algae pivot
   *
   * <p>TLDR: a special way of resetting encoders
   */
  public static Command homeEverything(Elevator elevator, AlgaeIntakePivot pivot) {
    return elevator
        .homeEncoder()
        .onlyIf(() -> !elevator.elevatorIsHomed())
        .andThen(
            pivot
                .homeMechanism()
                .onlyIf(() -> !pivot.pivotIsHomed())
                .deadlineFor(elevator.goToHeight(() -> ElevatorConstants.kElevatorDangerHeight)));
  }
}
