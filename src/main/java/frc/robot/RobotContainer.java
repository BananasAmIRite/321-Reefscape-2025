/* (C) Robolancers 2025 */
package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriverCommands;
import frc.robot.commands.HomingCommands;
import frc.robot.commands.ReefAlign;
import frc.robot.commands.StationAlign;
import frc.robot.subsystems.AlgaeSuperstructure;
import frc.robot.subsystems.AlgaeSuperstructure.AlgaeSetpoint;
import frc.robot.subsystems.CoralSuperstructure;
import frc.robot.subsystems.CoralSuperstructure.CoralScorerSetpoint;
import frc.robot.subsystems.SuperstructureVisualizer;
import frc.robot.subsystems.algaeIntakePivot.AlgaeIntakePivot;
import frc.robot.subsystems.algaeIntakeRollers.AlgaeIntakeRollers;
import frc.robot.subsystems.coralendeffector.CoralEndEffector;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
import frc.robot.subsystems.drivetrain.DrivetrainSim;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevatorarm.ElevatorArm;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.ReefPosition;
import java.util.function.DoubleSupplier;

@Logged
public class RobotContainer {
  // all subsystems
  private SwerveDrive drivetrain = SwerveDrive.create();
  private AlgaeIntakePivot algaePivot = AlgaeIntakePivot.create();
  private AlgaeIntakeRollers algaeRollers = AlgaeIntakeRollers.create();
  private CoralEndEffector coralEndEffector = CoralEndEffector.create();
  private ElevatorArm elevatorArm = ElevatorArm.create();
  private Elevator elevator = Elevator.create();

  // helper wrappers for the two main superstructure components of the robot
  @NotLogged
  private CoralSuperstructure coralSuperstructure =
      new CoralSuperstructure(elevator, elevatorArm, coralEndEffector);

  @NotLogged
  private AlgaeSuperstructure algaeSuperstructure =
      new AlgaeSuperstructure(algaePivot, algaeRollers);

  private Vision vision =
      Vision.create(
          // Java 21 pattern matching switch would be nice
          (drivetrain instanceof DrivetrainSim)
              ? ((DrivetrainSim) drivetrain)::getActualPose
              : drivetrain::getPose,
          visionEst ->
              drivetrain.addVisionMeasurement(
                  visionEst.estimate().estimatedPose.toPose2d(),
                  visionEst.estimate().timestampSeconds,
                  visionEst.stdDevs()),
          reefVisionEst ->
              drivetrain.addReefVisionMeasurement(
                  reefVisionEst.estimate().estimatedPose.toPose2d(),
                  reefVisionEst.estimate().timestampSeconds,
                  reefVisionEst.stdDevs()));

  // controllers
  private CommandXboxController driver = new CommandXboxController(0);
  private XboxController manipulator = new XboxController(1); // button board for manipulator

  private Trigger isSlowMode = new Trigger(() -> false);

  // drivetrain controls
  public DoubleSupplier driverForward =
      () ->
          -MathUtil.applyDeadband(driver.getLeftY(), DrivetrainConstants.kDriveDeadband)
              * (isSlowMode.getAsBoolean()
                  ? DrivetrainConstants.kSlowModeLinearVelocity.in(MetersPerSecond)
                  : DrivetrainConstants.kMaxLinearVelocity.in(MetersPerSecond));
  public DoubleSupplier driverStrafe =
      () ->
          -MathUtil.applyDeadband(driver.getLeftX(), DrivetrainConstants.kDriveDeadband)
              * (isSlowMode.getAsBoolean()
                  ? DrivetrainConstants.kSlowModeLinearVelocity.in(MetersPerSecond)
                  : DrivetrainConstants.kMaxLinearVelocity.in(MetersPerSecond));
  private DoubleSupplier driverTurn =
      () -> -MathUtil.applyDeadband(driver.getRightX(), DrivetrainConstants.kRotationDeadband) * 5;

  // robot queued states
  private ReefPosition queuedReefPosition = ReefPosition.NONE;
  private CoralScorerSetpoint queuedSetpoint = CoralScorerSetpoint.NEUTRAL;

  // visualizer for the whole superstructure
  private SuperstructureVisualizer stateVisualizer =
      new SuperstructureVisualizer(
          elevator::getHeight, elevatorArm::getAngle, algaePivot::getAngle);

  private boolean isDriverOverride = false;

  public RobotContainer() {
    // home everything on robot start
    RobotModeTriggers.disabled()
        .negate()
        .onTrue(HomingCommands.homeEverything(elevator, algaePivot));

    // drive
    drivetrain.setDefaultCommand(drivetrain.teleopDrive(driverForward, driverStrafe, driverTurn));

    // algae default commands (stalling rollers, default algae pivot setpoint)
    algaeRollers.setDefaultCommand(algaeRollers.stallIfHasAlgae());
    algaePivot.setDefaultCommand(algaePivot.goToAngle(() -> AlgaeSetpoint.NEUTRAL.getAlgaeAngle()));

    // elevator default commands
    elevator.setDefaultCommand(
        elevator.goToHeight(() -> CoralScorerSetpoint.NEUTRAL.getElevatorHeight()));
    elevatorArm.setDefaultCommand(
        elevatorArm.goToAngle(() -> CoralScorerSetpoint.NEUTRAL.getArmAngle()));
    coralEndEffector.setDefaultCommand(coralEndEffector.stallCoralIfDetected());

    // ensures that algae pivot doesn't hit elevator when it goes
    // when both are about to collide, move elevator out of the way until the algae pivot is out of
    // the collision zone
    new Trigger(algaePivot::inCollisionZone)
        .and(new Trigger(elevator::inCollisionZone))
        .onTrue(
            elevator
                .goToHeight(() -> ElevatorConstants.kElevatorDangerHeight)
                .until(new Trigger(algaePivot::inCollisionZone).negate()));

    configureBindings();
  }

  private void configureBindings() {
    // driver controls
    // score coral / flip off algae
    driver.y().toggleOnTrue(algaeSuperstructure.prepareClimb());
    driver.a().onTrue(algaeSuperstructure.climb());

    // --- CORAL AUTOMATED CONTROLS ---

    // coral feeding
    driver
        .rightBumper()
        .whileTrue(
            StationAlign.rotateToNearestStationTag(drivetrain, driverForward, driverStrafe)
                .alongWith(coralSuperstructure.feedCoral()));

    // WARNING: Ivan wrote the following below, please check. Not safe for those
    // who are sensitive and cannot handle programming crimes.

    // coral outtake
    // driver
    //     .rightTrigger()
    //     .whileTrue( // while right trigger is pressed:
    //         Commands.runOnce(() -> isDriverOverride = false)
    //             .andThen(
    //                 // either align to reef or coral based on how far we are away
    //                 // rotate to reef until we're close enough
    //                 ReefAlign.rotateToNearestReefTag(drivetrain, driverForward, driverStrafe)
    //                     .until(
    //                         () ->
    //                             ReefAlign.isWithinReefRange(
    //                                     drivetrain,
    //                                     ReefAlign
    //                                         .kMechanismDeadbandThreshold) // use mechanism
    // threshold
    //                                 // cuz we
    //                                 // wanna be close before aligning
    //                                 // in this case
    //                                 && Math.hypot(
    //                                         driverForward.getAsDouble(),
    // driverStrafe.getAsDouble())
    //                                     <= 0.05
    //                                 && !isDriverOverride)
    //                     .andThen(
    //                         // when we get close enough, align to reef, but only while we're
    // close
    //                         // enough
    //                         ReefAlign.alignToReef(drivetrain, () -> queuedReefPosition)
    //                             .onlyWhile(
    //                                 () ->
    //                                     ReefAlign.isWithinReefRange(
    //                                             drivetrain,
    // ReefAlign.kMechanismDeadbandThreshold)
    //                                         && Math.hypot(
    //                                                 driverForward.getAsDouble(),
    //                                                 driverStrafe.getAsDouble())
    //                                             <= 0.05
    //                                         &&
    //                                         // allow driver control to be taken back when
    //                                         // driverOverride becomes true
    //                                         !isDriverOverride))
    //                     // when we get far away, repeat the command
    //                     .repeatedly()
    //                     .alongWith( // and run the mechanism to where we need to go
    //                         coralSuperstructure
    //                             .goToSetpoint(
    //                                 // move arm up to avoid hitting reef until we get close to
    // reef
    //                                 () -> CoralScorerSetpoint.NEUTRAL.getElevatorHeight(),
    //                                 () -> ElevatorArmConstants.kPreAlignAngle)
    //                             .until(
    //                                 () ->
    //                                     coralSuperstructure.atTargetState()
    //                                         && ReefAlign.isWithinReefRange(
    //                                             drivetrain,
    // ReefAlign.kMechanismDeadbandThreshold))
    //                             .andThen(
    //                                 // move the elevator up but keep arm up
    //                                 coralSuperstructure
    //                                     .goToSetpoint(
    //                                         () -> queuedSetpoint.getElevatorHeight(),
    //                                         () -> ElevatorArmConstants.kPreAlignAngle)
    //                                     .until(() -> coralSuperstructure.atTargetState())
    //                                     // then move arm down to setpoint
    //                                     .andThen(
    //                                         coralSuperstructure.goToSetpoint(() ->
    // queuedSetpoint)))
    //                             // and only do this while we're in the zone (when we're not, we
    // will
    //                             // stay in the pre-alignment position)
    //                             .onlyWhile(
    //                                 () ->
    //                                     ReefAlign.isWithinReefRange(
    //                                             drivetrain,
    // ReefAlign.kMechanismDeadbandThreshold)
    //                                         && queuedSetpoint != CoralScorerSetpoint.NEUTRAL)
    //                             .repeatedly())));

    driver
        .rightTrigger()
        .whileTrue(
            Commands.runOnce(() -> isDriverOverride = false)
                .andThen(
                    DriverCommands.alignToReef(
                        drivetrain, driverForward, driverStrafe, () -> isDriverOverride))
                .repeatedly()
                .alongWith(
                    DriverCommands.runCoralSuperstructure(coralSuperstructure, queuedSetpoint))
                .onlyWhile(
                    () ->
                        ReefAlign.isWithinReefRange(
                                drivetrain, ReefAlign.kMechanismDeadbandThreshold)
                            && queuedSetpoint != CoralScorerSetpoint.NEUTRAL)
                .repeatedly());

    driver
        .rightTrigger()
        .onFalse( // for coral scoring
            DriverCommands.scoreCoral(coralSuperstructure)
                .onlyIf(
                    () ->
                        coralSuperstructure.atTargetState()
                            && queuedSetpoint != CoralScorerSetpoint.NEUTRAL
                            && !driver
                                .povLeft()
                                .getAsBoolean())); // only if we're at the target state and are
    // ready to score

    // --- ALGAE AUTOMATED CONTROLS ---

    // algae feeding
    driver.leftBumper().whileTrue(algaeSuperstructure.intakeAlgae());

    // algae outtake
    // driver
    //     .leftTrigger()
    //     .whileTrue( // while left trigger is pressed:
    //         Commands.runOnce(() -> isDriverOverride = false)
    //             .andThen(
    //                 // rotate to nearest processor unless conditions for full alignment are met
    //                 ProcessorAlign.rotateToNearestProcessor(drivetrain, driverForward,
    // driverStrafe)
    //                     .until(
    //                         () -> // conditions for full alignment: in range + driver not
    // pressing
    //                             // on stick + driver override is off
    //                             ProcessorAlign.isWithinProcessorRange(
    //                                     drivetrain, ProcessorAlign.kAlignmentDeadbandRange)
    //                                 && Math.hypot(
    //                                         driverForward.getAsDouble(),
    // driverStrafe.getAsDouble())
    //                                     <= 0.05
    //                                 && !isDriverOverride)
    //                     .andThen(
    //                         // conditions for full alignment are met, proceed with full alignment
    //                         ProcessorAlign.goToNearestAlign(drivetrain)
    //                             .onlyWhile(
    //                                 () ->
    //                                     !isDriverOverride
    //                                         && Math.hypot(
    //                                                 driverForward.getAsDouble(),
    //                                                 driverStrafe.getAsDouble())
    //                                             <= 0.05
    //                                         && ProcessorAlign.isWithinProcessorRange(
    //                                             drivetrain,
    //                                             ProcessorAlign.kAlignmentDeadbandRange)))
    //                     .repeatedly()
    //                     .alongWith(
    //                         algaeSuperstructure.goToSetpoint(
    //                             AlgaeSetpoint
    //                                 .OUTTAKE)))); // move algae intake to the correct setpoint

    driver
        .leftTrigger()
        .whileTrue(
            Commands.runOnce(() -> isDriverOverride = false)
                .andThen(
                    DriverCommands.algaeDriveAlign(
                        drivetrain, driverForward, driverStrafe, () -> isDriverOverride))
                .repeatedly()
                .alongWith(algaeSuperstructure.goToSetpoint(AlgaeSetpoint.OUTTAKE)));

    driver
        .leftTrigger()
        .onFalse( // when left trigger is let go
            DriverCommands.deployAlgae(drivetrain, algaeSuperstructure)
                .onlyIf(
                    () ->
                        algaeSuperstructure.atTargetState()
                            && !driver
                                .povLeft()
                                .getAsBoolean())); // only if algae intake is at outtake position

    // toggle driver override
    driver.povUp().onTrue(Commands.runOnce(() -> isDriverOverride = !isDriverOverride));

    /**
     * Preference 2:
     *
     * <p>Pressing right trigger down all the way performs translation-align/to-setpoint, while
     * pressing it slightly performs the rotation align
     *
     * <p>Driver has override over translation-align/to-setpoint
     */
    // new Trigger(() -> driver.getRightTriggerAxis() >= 0.8)
    //     .whileTrue(
    //         ReefAlign.alignToReef(drivetrain, () -> queuedReefPosition)
    //             .onlyWhile(
    //                 () ->
    //                     ReefAlign.isWithinReefRange(
    //                             drivetrain, ReefAlign.kMaxAlignmentDeadbandThreshold)
    //                         && Math.hypot(driverForward.getAsDouble(),
    // driverStrafe.getAsDouble())
    //                             <= 0.05)
    //             .asProxy()
    //             .repeatedly()
    //             .alongWith(
    //                 coralSuperstructure
    //                     .goToSetpoint(() -> queuedSetpoint)
    //                     .onlyWhile(
    //                         () ->
    //                             ReefAlign.isWithinReefRange(
    //                                 drivetrain, ReefAlign.kMechanismDeadbandThreshold))
    //                     .asProxy()
    //                     .repeatedly()));

    // new Trigger(() -> driver.getRightTriggerAxis() > 0.05 && driver.getRightTriggerAxis() < 0.8)
    //     .whileTrue(ReefAlign.rotateToNearestReefTag(drivetrain, driverForward, driverStrafe));

    // manip controls
    // 1 to 4 - right side L1-L4
    // 5 to 8 - left side L1-L4
    // 9 to 10 - algae low / high

    manipTrigger(1)
        .onTrue(
            Commands.runOnce(
                () -> {
                  queuedReefPosition = ReefPosition.RIGHT;
                  queuedSetpoint = CoralScorerSetpoint.L1;
                }));

    manipTrigger(2)
        .onTrue(
            Commands.runOnce(
                () -> {
                  queuedReefPosition = ReefPosition.RIGHT;
                  queuedSetpoint = CoralScorerSetpoint.L2;
                }));

    manipTrigger(3)
        .onTrue(
            Commands.runOnce(
                () -> {
                  queuedReefPosition = ReefPosition.RIGHT;
                  queuedSetpoint = CoralScorerSetpoint.L3;
                }));

    manipTrigger(4)
        .onTrue(
            Commands.runOnce(
                () -> {
                  queuedReefPosition = ReefPosition.RIGHT;
                  queuedSetpoint = CoralScorerSetpoint.L4;
                }));

    manipTrigger(5)
        .onTrue(
            Commands.runOnce(
                () -> {
                  queuedReefPosition = ReefPosition.LEFT;
                  queuedSetpoint = CoralScorerSetpoint.L1;
                }));

    manipTrigger(6)
        .onTrue(
            Commands.runOnce(
                () -> {
                  queuedReefPosition = ReefPosition.LEFT;
                  queuedSetpoint = CoralScorerSetpoint.L2;
                }));

    manipTrigger(7)
        .onTrue(
            Commands.runOnce(
                () -> {
                  queuedReefPosition = ReefPosition.LEFT;
                  queuedSetpoint = CoralScorerSetpoint.L3;
                }));

    manipTrigger(8)
        .onTrue(
            Commands.runOnce(
                () -> {
                  queuedReefPosition = ReefPosition.LEFT;
                  queuedSetpoint = CoralScorerSetpoint.L4;
                }));

    manipTrigger(9)
        .onTrue(
            Commands.runOnce(
                () -> {
                  queuedReefPosition = ReefPosition.ALGAE;
                  queuedSetpoint = CoralScorerSetpoint.ALGAE_LOW;
                }));

    manipTrigger(10)
        .onTrue(
            Commands.runOnce(
                () -> {
                  queuedReefPosition = ReefPosition.ALGAE;
                  queuedSetpoint = CoralScorerSetpoint.ALGAE_HIGH;
                }));
  }

  private Trigger manipTrigger(int button) {
    return new Trigger(() -> manipulator.getRawButton(button));
  }

  public Command getAutonomousCommand() {
    return Commands.none();
  }
}
