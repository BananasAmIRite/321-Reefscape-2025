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

public class DriverCommands {
  private static SwerveDrive drivetrain;

  // robot queued states
  private static Supplier<ReefPosition> queuedReefPosition;
  private static CoralScorerSetpoint queuedSetpoint = CoralScorerSetpoint.NEUTRAL;

  /**
   * Aligns the robot to the reef using AprilTags and reef positioning logic. Switches between
   * rotating and fully aligning when in range.
   */
  public static Command alignToReef(
      SwerveDrive drivetrain,
      DoubleSupplier driverForward,
      DoubleSupplier driverStrafe,
      BooleanSupplier isDriverOverride) {
    return ReefAlign.rotateToNearestReefTag(drivetrain, driverForward, driverStrafe)
        .until(
            () ->
                isAlignedToReef(drivetrain, driverForward, driverStrafe)
                    && !isDriverOverride.getAsBoolean())
        .andThen(
            ReefAlign.alignToReef(drivetrain, queuedReefPosition)
                .onlyWhile(
                    () ->
                        isAlignedToReef(drivetrain, driverForward, driverStrafe)
                            && !isDriverOverride.getAsBoolean()))
        .repeatedly();
  }

  /** Controlling coral superstructure */
  public static Command runCoralSuperstructure(
      CoralSuperstructure coralSuperstructure, CoralScorerSetpoint queuedSetpoint) {
    return coralSuperstructure
        .goToSetpoint(
            () -> CoralScorerSetpoint.NEUTRAL.getElevatorHeight(),
            () -> ElevatorArmConstants.kPreAlignAngle)
        .until(
            () ->
                coralSuperstructure.atTargetState()
                    && ReefAlign.isWithinReefRange(
                        drivetrain, ReefAlign.kMechanismDeadbandThreshold))
        .andThen(
            coralSuperstructure
                .goToSetpoint(
                    () -> queuedSetpoint.getElevatorHeight(),
                    () -> ElevatorArmConstants.kPreAlignAngle)
                .until(() -> coralSuperstructure.atTargetState())
                .andThen(coralSuperstructure.goToSetpoint(() -> queuedSetpoint)));
  }

  /** Checks if the robot is within reef alignment range. */
  private static boolean isAlignedToReef(
      SwerveDrive drivetrain, DoubleSupplier driverForward, DoubleSupplier driverStrafe) {
    return ReefAlign.isWithinReefRange(drivetrain, ReefAlign.kMechanismDeadbandThreshold)
        && Math.hypot(driverForward.getAsDouble(), driverStrafe.getAsDouble()) <= 0.05;
  }

  /** Scores coral on reef once driver releases trigger (after alignment) */
  public static Command scoreCoral(CoralSuperstructure coralSuperstructure) {
    return coralSuperstructure
        .goToSetpoint(() -> queuedSetpoint) // Ensure we are at the setpoint
        .alongWith(coralSuperstructure.outtakeCoral()) // Outtake coral
        .until(() -> !coralSuperstructure.hasCoral()) // until we don't have coral
        .withTimeout(1) // Max timeout of 1 second
        .andThen( // move arm up and go back down (only if we're already at the scoring setpoint
            // state)
            coralSuperstructure
                .goToSetpoint(
                    () -> CoralScorerSetpoint.NEUTRAL.getElevatorHeight(),
                    () -> ElevatorArmConstants.kPreAlignAngle) // Move arm up after scoring
                .until(coralSuperstructure::atTargetState)); // and then resume default command
  }

  public static Command algaeDriveAlign(
      SwerveDrive drivetrain,
      DoubleSupplier driverForward,
      DoubleSupplier driverStrafe,
      BooleanSupplier isDriverOverride) {
    return ProcessorAlign.rotateToNearestProcessor(drivetrain, driverForward, driverStrafe)
        .until(
            () ->
                ProcessorAlign.isWithinProcessorRange(
                        drivetrain, ProcessorAlign.kAlignmentDeadbandRange)
                    && Math.hypot(driverForward.getAsDouble(), driverStrafe.getAsDouble()) <= 0.05)
        .andThen(
            ProcessorAlign.goToNearestAlign(drivetrain)
                .onlyWhile(
                    () ->
                        Math.hypot(driverForward.getAsDouble(), driverStrafe.getAsDouble()) <= 0.05
                            && ProcessorAlign.isWithinProcessorRange(
                                drivetrain, ProcessorAlign.kAlignmentDeadbandRange)));
  }

  public static Command deployAlgae(
      SwerveDrive drivetrain, AlgaeSuperstructure algaeSuperstructure) {
    return algaeSuperstructure
        .goToSetpoint(AlgaeSetpoint.OUTTAKE) // score until we don't have algae or with 1s timeout
        .alongWith(algaeSuperstructure.outtakeAlgae())
        .until(() -> !algaeSuperstructure.hasAlgae())
        .withTimeout(1); // only if algae intake is at outtake position
  }
}
