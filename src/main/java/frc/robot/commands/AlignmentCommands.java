/* (C) Robolancers 2025 */
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeSuperstructure;
import frc.robot.subsystems.AlgaeSuperstructure.AlgaeSetpoint;
import frc.robot.subsystems.CoralSuperstructure;
import frc.robot.subsystems.CoralSuperstructure.CoralScorerSetpoint;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.elevatorarm.ElevatorArmConstants;
import frc.robot.util.ReefPosition;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class AlignmentCommands {

  /**
   * Aligns the robot to the reef using AprilTags and reef positioning logic. Switches between
   * rotating and fully aligning when in range.
   */
  public static Command autoAlignReef(
      Supplier<ReefPosition> reefPos,
      SwerveDrive drivetrain,
      DoubleSupplier driverForward,
      DoubleSupplier driverStrafe,
      BooleanSupplier isDriverOverride) {
    return selectCommand(
        ReefAlign.rotateToNearestReefTag(drivetrain, driverForward, driverStrafe),
        ReefAlign.alignToReef(drivetrain, reefPos),
        () ->
            isAlignedToReef(drivetrain, driverForward, driverStrafe)
                && !isDriverOverride.getAsBoolean());
  }

  /** Controlling coral superstructure */
  public static Command autoAimSuperstructureReef(
      SwerveDrive drivetrain,
      CoralSuperstructure coralSuperstructure,
      Supplier<CoralScorerSetpoint> queuedSetpoint) {
    return coralSuperstructure
        .goToSetpoint(
            // move arm up to avoid hitting reef until we get close to reef
            () -> CoralScorerSetpoint.NEUTRAL.getElevatorHeight(),
            () -> ElevatorArmConstants.kPreAlignAngle)
        .until(
            () ->
                coralSuperstructure.atTargetState()
                    && ReefAlign.isWithinReefRange(
                        drivetrain, ReefAlign.kMechanismDeadbandThreshold))
        .andThen(
            // move the elevator up but keep arm up
            coralSuperstructure
                .goToSetpoint(
                    () -> queuedSetpoint.get().getElevatorHeight(),
                    () -> ElevatorArmConstants.kPreAlignAngle)
                .until(() -> coralSuperstructure.atTargetState())
                // then move arm down to setpoint
                .andThen(coralSuperstructure.goToSetpoint(queuedSetpoint)))
        // and only do this while we're in the zone (when we're not, we will
        // stay in the pre-alignment position)
        .onlyWhile(
            () ->
                ReefAlign.isWithinReefRange(drivetrain, ReefAlign.kMechanismDeadbandThreshold)
                    && queuedSetpoint.get() != CoralScorerSetpoint.NEUTRAL)
        .repeatedly();
  }

  /** Checks if the robot is within reef alignment range. */
  private static boolean isAlignedToReef(
      SwerveDrive drivetrain, DoubleSupplier driverForward, DoubleSupplier driverStrafe) {
    return ReefAlign.isWithinReefRange(drivetrain, ReefAlign.kMechanismDeadbandThreshold)
        && Math.hypot(driverForward.getAsDouble(), driverStrafe.getAsDouble()) <= 0.05;
  }

  /** Scores coral on reef once driver releases trigger (after alignment) */
  public static Command autoScoreCoral(
      CoralSuperstructure coralSuperstructure,
      Supplier<CoralScorerSetpoint> queuedSetpoint,
      BooleanSupplier shouldScore) {
    return coralSuperstructure
        .goToSetpoint(queuedSetpoint) // Ensure we are at the setpoint
        .alongWith(coralSuperstructure.outtakeCoral()) // Outtake coral
        .until(() -> !coralSuperstructure.hasCoral()) // until we don't have coral
        .withTimeout(1) // Max timeout of 1 second
        .andThen( // move arm up and go back down (only if we're already at the scoring setpoint
            // state)
            coralSuperstructure
                .goToSetpoint(
                    () -> CoralScorerSetpoint.NEUTRAL.getElevatorHeight(),
                    () -> ElevatorArmConstants.kPreAlignAngle) // Move arm up after scoring
                .until(coralSuperstructure::atTargetState))
        .onlyIf(
            () ->
                coralSuperstructure.atTargetState()
                    && queuedSetpoint.get() != CoralScorerSetpoint.NEUTRAL
                    && shouldScore.getAsBoolean()); // and then resume default command
  }

  public static Command autoAlignAlgae(
      SwerveDrive drivetrain,
      DoubleSupplier driverForward,
      DoubleSupplier driverStrafe,
      BooleanSupplier isDriverOverride) {
    return selectCommand(
        ProcessorAlign.rotateToNearestProcessor(drivetrain, driverForward, driverStrafe),
        ProcessorAlign.goToNearestAlign(drivetrain),
        () ->
            Math.hypot(driverForward.getAsDouble(), driverStrafe.getAsDouble()) <= 0.05
                && ProcessorAlign.isWithinProcessorRange(
                    drivetrain, ProcessorAlign.kAlignmentDeadbandRange));
  }

  public static Command autoOuttakeAlgae(
      SwerveDrive drivetrain,
      AlgaeSuperstructure algaeSuperstructure,
      BooleanSupplier shouldScore) {
    return algaeSuperstructure
        .goToSetpoint(AlgaeSetpoint.OUTTAKE) // score until we don't have algae or with 1s timeout
        .alongWith(algaeSuperstructure.outtakeAlgae())
        .until(() -> !algaeSuperstructure.hasAlgae())
        .withTimeout(1)
        .onlyIf(
            () ->
                algaeSuperstructure.atTargetState()
                    && shouldScore.getAsBoolean()); // only if algae intake is at outtake position
  }

  // select between two running commands repeatedly, depending on the value of the condition
  private static Command selectCommand(
      Command cmdIfFalse, Command cmdIfTrue, BooleanSupplier condition) {
    return cmdIfFalse.until(condition).andThen(cmdIfTrue.onlyWhile(condition)).repeatedly();
  }
}
