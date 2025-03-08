/* (C) Robolancers 2025 */
package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.auto.AutomaticAutonomousMaker3000;
import frc.robot.commands.ReefAlign;
import frc.robot.commands.StationAlign;
import frc.robot.subsystems.AlgaeSuperstructure;
import frc.robot.subsystems.CoralSuperstructure;
import frc.robot.subsystems.CoralSuperstructure.CoralScorerSetpoint;
import frc.robot.subsystems.SuperstructureVisualizer;
import frc.robot.subsystems.algaeIntakePivot.AlgaeIntakePivot;
import frc.robot.subsystems.algaeIntakeRollers.AlgaeIntakeRollers;
import frc.robot.subsystems.climbtest.ClimbTest;
import frc.robot.subsystems.coralendeffector.CoralEndEffector;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
import frc.robot.subsystems.drivetrain.DrivetrainSim;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevatorarm.ElevatorArm;
import frc.robot.subsystems.elevatorarm.ElevatorArmConstants;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.ReefPosition;
import java.util.function.DoubleSupplier;

@Logged
public class RobotContainer {
  private SwerveDrive drivetrain = SwerveDrive.create();
  private AlgaeIntakePivot algaePivot = AlgaeIntakePivot.disable();
  private AlgaeIntakeRollers algaeRollers = AlgaeIntakeRollers.disable();
  private CoralEndEffector coralEndEffector = CoralEndEffector.disable();
  private ElevatorArm elevatorArm = ElevatorArm.disable();
  private Elevator elevator = Elevator.disable();

  private ClimbTest climbTest = new ClimbTest();

  private CoralSuperstructure coralSuperstructure =
      new CoralSuperstructure(elevator, elevatorArm, coralEndEffector);
  private AlgaeSuperstructure algaeSuperstructure =
      new AlgaeSuperstructure(algaePivot, algaeRollers);

  private AutomaticAutonomousMaker3000 automaker =
      new AutomaticAutonomousMaker3000(drivetrain, coralSuperstructure);

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

  private CommandXboxController driver = new CommandXboxController(0);
  private XboxController manipulator = new XboxController(1);

  private Trigger isSlowMode = driver.leftBumper();

  private DoubleSupplier driverForward =
      () ->
          -MathUtil.applyDeadband(Math.hypot(driver.getLeftY(), driver.getLeftX()), 0.05)
              * Math.cos(Math.atan2(driver.getLeftX(), driver.getLeftY()))
              * (isSlowMode.getAsBoolean()
                  ? 1.5
                  : DrivetrainConstants.kMaxLinearVelocity.in(MetersPerSecond));
  private DoubleSupplier driverStrafe =
      () ->
          -MathUtil.applyDeadband(Math.hypot(driver.getLeftY(), driver.getLeftX()), 0.05)
              * Math.sin(Math.atan2(driver.getLeftX(), driver.getLeftY()))
              * (isSlowMode.getAsBoolean()
                  ? 1.5
                  : DrivetrainConstants.kMaxLinearVelocity.in(MetersPerSecond));
  private DoubleSupplier driverTurn = () -> -MathUtil.applyDeadband(driver.getRightX(), 0.05) * 5;

  // robot queued states
  private ReefPosition queuedReefPosition = ReefPosition.RIGHT;
  private CoralScorerSetpoint queuedSetpoint = CoralScorerSetpoint.L4;

  private SuperstructureVisualizer stateVisualizer =
      new SuperstructureVisualizer(
          () -> elevator.getHeight(), () -> elevatorArm.getAngle(), () -> algaePivot.getAngle());

  private Leds leds = new Leds();
  private AddressableLEDSim ledSim = new AddressableLEDSim(leds.strip);
  private boolean isDriverOverride = false;

  private Trigger isAlgaeSetpoint =
      new Trigger(
          () ->
              queuedSetpoint == CoralScorerSetpoint.ALGAE_LOW
                  || queuedSetpoint == CoralScorerSetpoint.ALGAE_HIGH);
  private Trigger isCoralSetpoint =
      new Trigger(
          () ->
              queuedSetpoint == CoralScorerSetpoint.L1
                  || queuedSetpoint == CoralScorerSetpoint.L2
                  || queuedSetpoint == CoralScorerSetpoint.L3
                  || queuedSetpoint == CoralScorerSetpoint.L4);

  public RobotContainer() {

    // reset elevator arm encoder on robot enable
    RobotModeTriggers.disabled().negate().onTrue(elevatorArm.seedEncoder());

    // home everything on robot start
    RobotModeTriggers.disabled()
        .negate()
        .onTrue(elevator.homeEncoder().onlyIf(() -> !elevator.elevatorIsHomed()));

    // drive
    drivetrain.setDefaultCommand(drivetrain.teleopDrive(driverForward, driverStrafe, driverTurn));

    // full-featured default commnds
    // algaeRollers.setDefaultCommand(algaeRollers.stallIfHasAlgae());
    // algaePivot.setDefaultCommand(algaePivot.goToAngle(() ->
    // AlgaeSetpoint.NEUTRAL.getAlgaeAngle()));
    elevator.setDefaultCommand(
        elevator.goToHeight(() -> CoralScorerSetpoint.NEUTRAL.getElevatorHeight()));
    elevatorArm.setDefaultCommand(
        elevatorArm.goToAngle(() -> CoralScorerSetpoint.NEUTRAL.getArmAngle()));
    coralEndEffector.setDefaultCommand(coralEndEffector.stallCoralIfDetected());

    // testing default commands
    algaeRollers.setDefaultCommand(algaeRollers.setMechanismVoltage(() -> Volts.zero()));
    algaePivot.setDefaultCommand(algaePivot.setMechanismVoltage(() -> Volts.zero()));
    // elevator.setDefaultCommand(elevator.setVoltage(() -> Volts.zero()));
    // elevatorArm.setDefaultCommand(elevatorArm.runVolts(() -> Volts.zero()));
    // coralEndEffector.setDefaultCommand(coralEndEffector.runVolts(() -> Volts.of(1)));

    climbTest.setDefaultCommand(climbTest.setMechanismVoltage(() -> Volts.zero()));

    leds.setDefaultCommand(leds.updateLeds());

    // when both are about to collide, move elevator out of the way until the algae pivot is out of
    // the collision zone
    // new Trigger(algaePivot::inCollisionZone)
    //     .and(new Trigger(elevator::inCollisionZone))
    //     .onTrue(
    //         elevator
    //             .goToHeight(() -> ElevatorConstants.kElevatorDangerHeight.plus(Meters.of(0.1)))
    //             .until(new Trigger(algaePivot::inCollisionZone).negate()));

    // configureBindings();
    configureTuningBindings();
  }

  private double volts = 0;

  private void configureTuningBindings() {

    // climb!
    // driver.y().toggleOnTrue(algaeSuperstructure.prepareClimb());

    driver.b().toggleOnTrue(coralSuperstructure.goToSetpoint(() -> CoralScorerSetpoint.CLIMB));
    driver.y().whileTrue(climbTest.setMechanismVoltage(() -> Volts.of(1)));
    driver.a().whileTrue(climbTest.setMechanismVoltage(() -> Volts.of(-1)));
    driver.x().whileTrue(climbTest.setMechanismVoltage(() -> Volts.of(volts)));

    driver
        .leftBumper()
        .onTrue(
            Commands.runOnce(
                () -> {
                  volts += 0.1;
                  System.out.println("Changing volts to: " + volts);
                }));

    driver
        .rightBumper()
        .onTrue(
            Commands.runOnce(
                () -> {
                  volts -= 0.1;
                  System.out.println("Changing volts to: " + volts);
                }));

    driver.leftTrigger().onTrue(Commands.runOnce(() -> climbTest.lockServo()));
    driver.rightTrigger().onTrue(Commands.runOnce(() -> climbTest.unlockServo()));

    // driver.a().whileTrue(ReefAlign.tuneAlignment(drivetrain));

    // driver.b().whileTrue(coralSuperstructure.feedCoral());

    // driver.leftBumper().whileTrue(elevator.setVoltage(() -> Volts.of(1)));
    // driver.rightBumper().whileTrue(elevator.setVoltage(() -> Volts.of(-1)));

    // driver.povLeft().whileTrue(elevatorArm.runVolts(() -> Volts.of(1)));
    // driver.povRight().whileTrue(elevatorArm.runVolts(() -> Volts.of(-1)));

    // tune elevator
    // driver.a().whileTrue(elevator.tune());

    // tune elevator arm
    // driver.a().whileTrue(elevatorArm.tune());

    // find arm setpoints
    // driver.y().whileTrue(coralSuperstructure.tune());
    // driver.leftBumper().whileTrue(coralSuperstructure.feedCoral());
    // driver.rightBumper().whileTrue(coralEndEffector.outtakeCoral());

    // alignment testing (no arm)
    // driver.a().whileTrue(ReefAlign.rotateToNearestReefTag(drivetrain, driverForward,
    // driverStrafe));
    // driver.b().whileTrue(ReefAlign.alignToReef(drivetrain, () -> ReefPosition.LEFT));

    // test algae intake
    // driver.b().whileTrue(algaeSuperstructure.intakeAlgae());
    // driver.a().whileTrue(algaeSuperstructure.outtakeAlgae());
  }

  private void configureBindings() {
    // driver controls
    // score coral / flip off algae
    // driver.y().toggleOnTrue(algaeSuperstructure.prepareClimb());
    // driver.a().onTrue(algaeSuperstructure.climb());

    driver.y().whileTrue(coralSuperstructure.feedCoral());

    // --- CORAL AUTOMATED CONTROLS ---

    // coral feeding
    driver
        .rightBumper()
        .whileTrue(
            StationAlign.rotateToNearestStationTag(drivetrain, driverForward, driverStrafe)
                .onlyWhile(() -> StationAlign.getStationDistance(drivetrain) < 2)
                .andThen(drivetrain.teleopDrive(driverForward, driverStrafe, driverTurn))
                .until(() -> StationAlign.getStationDistance(drivetrain) < 2)
                .repeatedly()
                .alongWith(coralSuperstructure.feedCoral().asProxy().repeatedly()));

    // coral outtake
    driver
        .rightTrigger()
        .and(isCoralSetpoint)
        .whileTrue( // while right trigger is pressed:
            Commands.runOnce(() -> isDriverOverride = false)
                .andThen(
                    // either align to reef or coral based on how far we are away rotate to reef
                    // until we're close enough
                    ReefAlign.rotateToNearestReefTag(drivetrain, driverForward, driverStrafe)
                        .until(
                            () ->
                                ReefAlign.isWithinReefRange(
                                        drivetrain, ReefAlign.kMechanismDeadbandThreshold)
                                    // use mechanism threshold cuz we wanna be close before aligning
                                    // in this case
                                    && Math.hypot(
                                            driverForward.getAsDouble(), driverStrafe.getAsDouble())
                                        <= 0.05
                                    && !isDriverOverride)
                        .andThen(
                            // when we get close enough, align to reef, but only while we're
                            // close enough
                            ReefAlign.alignToReef(drivetrain, () -> queuedReefPosition)
                                // .until(drivetrain::atPoseSetpoint)
                                // .andThen(
                                //     ReefAlign.rotateToNearestReefTag(drivetrain, driverForward,
                                // driverStrafe))
                                .onlyWhile(
                                    () ->
                                        ReefAlign.isWithinReefRange(
                                                drivetrain, ReefAlign.kMechanismDeadbandThreshold)
                                            && Math.hypot(
                                                    driverForward.getAsDouble(),
                                                    driverStrafe.getAsDouble())
                                                <= 0.05
                                            &&
                                            // allow driver control to be taken back when
                                            // driverOverride becomes true
                                            !isDriverOverride))
                        // when we get far away, repeat the command
                        .repeatedly()
                        .alongWith( // and run the mechanism to where we need to go
                            coralSuperstructure
                                .goToSetpoint(
                                    // move arm up to avoid hitting reef until we get close to reef
                                    () -> CoralScorerSetpoint.NEUTRAL.getElevatorHeight(),
                                    () -> ElevatorArmConstants.kPreAlignAngle)
                                .until(
                                    () ->
                                        coralSuperstructure
                                                .getElevator()
                                                .atHeight(
                                                    CoralScorerSetpoint.NEUTRAL.getElevatorHeight())
                                            && ReefAlign.isWithinReefRange(
                                                drivetrain, ReefAlign.kMechanismDeadbandThreshold))
                                .andThen(
                                    // move the elevator up but keep arm up
                                    coralSuperstructure
                                        .goToSetpoint(
                                            () -> queuedSetpoint.getElevatorHeight(),
                                            () -> ElevatorArmConstants.kPreAlignAngle)
                                        .until(
                                            () ->
                                                coralSuperstructure
                                                    .getElevator()
                                                    .atHeight(queuedSetpoint.getElevatorHeight()))
                                        // then move arm down to setpoint
                                        .andThen(
                                            coralSuperstructure.goToSetpoint(() -> queuedSetpoint)))
                                // and only do this while we're in the zone (when we're not, we will
                                // stay in the pre-alignment position)
                                .onlyWhile(
                                    () ->
                                        ReefAlign.isWithinReefRange(
                                                drivetrain, ReefAlign.kMechanismDeadbandThreshold)
                                            && queuedSetpoint != CoralScorerSetpoint.NEUTRAL)
                                .repeatedly())));

    driver
        .rightTrigger()
        .onFalse( // for coral scoring
            coralSuperstructure
                .goToSetpoint(() -> queuedSetpoint) // ensure we're at the setpoint
                .alongWith(coralSuperstructure.outtakeCoral())
                .onlyIf(
                    () ->
                        coralSuperstructure.atTargetState(queuedSetpoint)
                            && queuedSetpoint != CoralScorerSetpoint.NEUTRAL
                            && !driver.povLeft().getAsBoolean()
                            && ReefAlign.isWithinReefRange(
                                drivetrain, ReefAlign.kMechanismDeadbandThreshold)
                            && isCoralSetpoint.getAsBoolean()) // and outtake coral
                // .until(() -> !coralSuperstructure.hasCoral()) // until we don't have coral
                .withTimeout(0.5) // timeout at 1 second
                .andThen(
                    // move arm up and go back down (only if we're already at the scoring setpoint
                    // state)
                    coralSuperstructure
                        .goToSetpoint(
                            () -> CoralScorerSetpoint.NEUTRAL.getElevatorHeight(),
                            () -> ElevatorArmConstants.kPreAlignAngle)
                        .until(
                            () ->
                                coralSuperstructure
                                    .getElevator()
                                    .atHeight(CoralScorerSetpoint.NEUTRAL.getElevatorHeight()))
                        .onlyIf(
                            () ->
                                !coralSuperstructure
                                    .getElevator()
                                    .atHeight(
                                        CoralScorerSetpoint.NEUTRAL
                                            .getElevatorHeight()))) // and then resume default
                // command
                // only if we're at the target state and are ready to score
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    // --- CORAL MANUAL CONTROLS ---
    driver
        .leftTrigger()
        .and(isCoralSetpoint)
        .whileTrue(
            coralSuperstructure
                .goToSetpoint(
                    // move arm up to avoid hitting reef until we get close to reef
                    () -> CoralScorerSetpoint.NEUTRAL.getElevatorHeight(),
                    () -> ElevatorArmConstants.kPreAlignAngle)
                .until(
                    () ->
                        coralSuperstructure
                            .getElevator()
                            .atHeight(CoralScorerSetpoint.NEUTRAL.getElevatorHeight()))
                .andThen(
                    // move the elevator up but keep arm up
                    coralSuperstructure
                        .goToSetpoint(
                            () -> queuedSetpoint.getElevatorHeight(),
                            () -> ElevatorArmConstants.kPreAlignAngle)
                        .until(
                            () ->
                                coralSuperstructure
                                    .getElevator()
                                    .atHeight(queuedSetpoint.getElevatorHeight()))
                        // then move arm down to setpoint
                        .andThen(coralSuperstructure.goToSetpoint(() -> queuedSetpoint)))
            // and only do this while we're in the zone (when we're not, we will
            // stay in the pre-alignment position)
            );

    driver
        .leftTrigger()
        .onFalse(
            coralSuperstructure
                .goToSetpoint(() -> queuedSetpoint) // ensure we're at the setpoint
                .alongWith(coralSuperstructure.outtakeCoral())
                .onlyIf(
                    () ->
                        coralSuperstructure.atTargetState(queuedSetpoint)
                            && queuedSetpoint != CoralScorerSetpoint.NEUTRAL
                            && !driver.povLeft().getAsBoolean()
                            && isCoralSetpoint.getAsBoolean()) // and outtake coral
                // .until(() -> !coralSuperstructure.hasCoral()) // until we don't have coral
                .withTimeout(0.5) // timeout at 1 second
                .andThen(
                    // move arm up and go back down (only if we're already at the scoring setpoint
                    // state)
                    coralSuperstructure
                        .goToSetpoint(
                            () -> CoralScorerSetpoint.NEUTRAL.getElevatorHeight(),
                            () -> ElevatorArmConstants.kPreAlignAngle)
                        .until(
                            () ->
                                coralSuperstructure
                                    .getElevator()
                                    .atHeight(CoralScorerSetpoint.NEUTRAL.getElevatorHeight()))
                        .onlyIf(
                            () ->
                                !coralSuperstructure
                                    .getElevator()
                                    .atHeight(
                                        CoralScorerSetpoint.NEUTRAL
                                            .getElevatorHeight()))) // and then resume default
                // command
                // only if we're at the target state and are ready to score

                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    // --- DEALGAEFYING ---
    driver
        .b()
        .and(isAlgaeSetpoint)
        .whileTrue(
            ReefAlign.rotateToNearestReefTag(drivetrain, driverForward, driverStrafe)
                .alongWith(
                    coralSuperstructure
                        .goToSetpoint(
                            () -> CoralScorerSetpoint.NEUTRAL.getElevatorHeight(),
                            () -> ElevatorArmConstants.kPreAlignAngle)
                        .until(
                            () ->
                                coralSuperstructure
                                    .getElevator()
                                    .atHeight(CoralScorerSetpoint.NEUTRAL.getElevatorHeight()))
                        .andThen(coralSuperstructure.goToSetpoint(() -> queuedSetpoint)))
                .alongWith(coralSuperstructure.knockAlgae()));

    // --- ALGAE AUTOMATED CONTROLS ---

    // // algae feeding
    // driver.leftBumper().whileTrue(algaeSuperstructure.intakeAlgae());

    // // algae outtake
    // driver
    //     .leftTrigger()
    //     .whileTrue( // while left trigger is pressed:
    //         Commands.runOnce(() -> isDriverOverride = false)
    //             .andThen(
    //                 // rotate to nearest processor unless conditions for full alignment are met
    //                 ProcessorAlign.rotateToNearestProcessor(drivetrain, driverForward,
    // driverStrafe)
    //                     .until(
    //                         () ->
    //                             // conditions for full alignment: in range + driver not pressing
    // on
    //                             // stick + driver override is off
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

    // driver
    //     .leftTrigger()
    //     .onFalse(
    //         // when left trigger is let go
    //         algaeSuperstructure
    //             .goToSetpoint(AlgaeSetpoint.OUTTAKE)
    //             // score until we don't have algae or with 1s timeout
    //             .alongWith(algaeSuperstructure.outtakeAlgae())
    //             .until(() -> !algaeSuperstructure.hasAlgae())
    //             .withTimeout(1)
    //             // only if algae intake is at outtake position
    //             .onlyIf(
    //                 () -> algaeSuperstructure.atTargetState() &&
    // !driver.povLeft().getAsBoolean()));

    // toggle driver override
    driver.povUp().onTrue(Commands.runOnce(() -> isDriverOverride = !isDriverOverride));

    // manip controls
    // 1 to 4 - right side L1-L4
    // 5 to 8 - left side L1-L4
    // 9 to 10 - algae low / high

    manipTrigger(2)
        .onTrue(
            Commands.runOnce(
                () -> {
                  queuedReefPosition = ReefPosition.RIGHT;
                  queuedSetpoint = CoralScorerSetpoint.L1;
                }));

    manipTrigger(3)
        .onTrue(
            Commands.runOnce(
                () -> {
                  queuedReefPosition = ReefPosition.RIGHT;
                  queuedSetpoint = CoralScorerSetpoint.L2;
                }));

    manipTrigger(8)
        .onTrue(
            Commands.runOnce(
                () -> {
                  queuedReefPosition = ReefPosition.RIGHT;
                  queuedSetpoint = CoralScorerSetpoint.L3;
                }));

    manipTrigger(7)
        .onTrue(
            Commands.runOnce(
                () -> {
                  queuedReefPosition = ReefPosition.RIGHT;
                  queuedSetpoint = CoralScorerSetpoint.L4;
                }));

    manipTrigger(1)
        .onTrue(
            Commands.runOnce(
                () -> {
                  queuedReefPosition = ReefPosition.LEFT;
                  queuedSetpoint = CoralScorerSetpoint.L1;
                }));

    manipTrigger(4)
        .onTrue(
            Commands.runOnce(
                () -> {
                  queuedReefPosition = ReefPosition.LEFT;
                  queuedSetpoint = CoralScorerSetpoint.L2;
                }));

    manipTrigger(6)
        .onTrue(
            Commands.runOnce(
                () -> {
                  queuedReefPosition = ReefPosition.LEFT;
                  queuedSetpoint = CoralScorerSetpoint.L3;
                }));

    manipTrigger(5)
        .onTrue(
            Commands.runOnce(
                () -> {
                  queuedReefPosition = ReefPosition.LEFT;
                  queuedSetpoint = CoralScorerSetpoint.L4;
                }));

    new Trigger(() -> manipulator.getPOV() == 270)
        .onTrue(
            Commands.runOnce(
                () -> {
                  queuedReefPosition = ReefPosition.ALGAE;
                  queuedSetpoint = CoralScorerSetpoint.ALGAE_LOW;
                }));

    new Trigger(() -> manipulator.getPOV() == 180)
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
    return automaker.getStoredAuto();
  }
}
