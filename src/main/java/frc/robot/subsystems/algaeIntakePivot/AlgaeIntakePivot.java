/* (C) Robolancers 2025 */
package frc.robot.subsystems.algaeIntakePivot;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import frc.robot.subsystems.AlgaeSuperstructure.AlgaeSetpoint;
import frc.robot.util.TunableConstant;
import java.util.function.Supplier;

/* The mechanism that intakes algae low and pivots back
 * to hang from the deep cage
 */
@Logged
public class AlgaeIntakePivot extends SubsystemBase {

  private AlgaeIntakePivotIO io;
  private AlgaeIntakePivotInputs inputs;

  private PIDController algaeIntakeClimbController; // pid controller, used for pivot
  private ArmFeedforward feedForward; // feed forward for pivot

  private AlgaeIntakePivotConfig config;

  private Angle targetAngle = AlgaeIntakePivotConstants.kPivotStartingAngle;

  private boolean isHomed = false;

  public AlgaeIntakePivot(AlgaeIntakePivotIO io, AlgaeIntakePivotConfig config) {
    this.io = io;
    this.inputs = new AlgaeIntakePivotInputs(); // sets io, inputs, and config
    this.config = config;

    algaeIntakeClimbController = new PIDController(config.kP(), config.kI(), config.kD());
    feedForward = new ArmFeedforward(0, config.kG(), 0); // creates pid controller and feed forward

    algaeIntakeClimbController.setTolerance(
        AlgaeIntakePivotConstants.kControllerTolerance.in(Degrees));
  }

  /* Tune PID and feed forward constants(kP, kI, kD, kG) live on smart dashboard / ascope
  so that we dont have to re run the code every time we change on of them. */
  public Command tune() {
    TunableConstant kP = new TunableConstant("/AlgaeIntakeClimbPivot/kP", config.kP());
    TunableConstant kI = new TunableConstant("/AlgaeIntakeClimbPivot/kI", config.kI());
    TunableConstant kD = new TunableConstant("/AlgaeIntakeClimbPivot/kD", config.kD());
    TunableConstant kG = new TunableConstant("/AlgaeIntakeClimbPivot/kG", config.kG());
    TunableConstant desiredAngle = new TunableConstant("/AlgaeIntakeClimbPivot/desiredAngle", 0);

    return run(
        () -> {
          this.algaeIntakeClimbController.setPID(kP.get(), kI.get(), kD.get());
          this.feedForward = new ArmFeedforward(0, kG.get(), 0);
          goToAngle(Degrees.of(desiredAngle.get()));
        });
  }

  public static AlgaeIntakePivot create() {
    return RobotBase.isReal() // TODO: possibly change from spark to kraken
        ? new AlgaeIntakePivot(
            new AlgaeIntakePivotIOSpark(),
            AlgaeIntakePivotIOSpark
                .config) // creates real mechanism if the code is running on a robot
        : new AlgaeIntakePivot(
            new AlgaeIntakePivotIOSim(),
            AlgaeIntakePivotIOSim
                .config); // creates a sim mechanism if the code is not on a real robot
  }

  // creates placeholder implementation to disable robot
  public static AlgaeIntakePivot disable() {
    return new AlgaeIntakePivot(new AlgaeIntakePivotIOIdeal(), AlgaeIntakePivotIOIdeal.config);
  }

  // get to a desired angle by setting pivot voltage to sum of calculated pid and feedforward
  public void goToAngle(Angle desiredAngle) {
    Voltage desiredVoltage =
        Volts.of(
            feedForward.calculate(desiredAngle.in(Radians), 0)
                + algaeIntakeClimbController.calculate(
                    inputs.pivotAngle.in(Degrees), desiredAngle.in(Degrees)));

    this.targetAngle = desiredAngle;

    io.setPivotVoltage(desiredVoltage);
  }

  public Command goToAngle(Supplier<Angle> desiredAngle) {
    return run(
        () -> {
          goToAngle(desiredAngle.get());
        });
  }

  public Command goToAngle(AlgaeSetpoint setpoint) {
    return goToAngle(() -> setpoint.getAlgaeAngle());
  }

  public Command outtakePosition() {
    return goToAngle(AlgaeSetpoint.OUTTAKE);
  }

  public Command intakePosition() {
    return goToAngle(AlgaeSetpoint.INTAKE);
  }

  // flips intake to the floor in preparation for climb, which consists
  // of the mechanism pivoting back and clamping onto the cage
  public Command climbFloorPosition() {
    return goToAngle(AlgaeSetpoint.CLIMB_PREP);
  }

  // mechanism clamps onto cage by rotating via pivot
  public Command climb() {
    return run(
        () -> {
          if (inputs.pivotAngle.in(Degrees)
              > AlgaeIntakePivotConstants.kPivotClimbThreshold.in(Degrees)) {
            io.setPivotVoltage(AlgaeIntakePivotConstants.kPivotClimbVoltage);
          } else {
            io.setPivotVoltage(Volts.of(0));
          }
        });
  }

  public Command homeMechanism() {
    return setMechanismVoltage(() -> AlgaeIntakePivotConstants.kHomingVoltage)
        .until(
            () ->
                (inputs.pivotCurrent.in(Amp)
                        > AlgaeIntakePivotConstants.kHomingCurrentThreshold.in(Amp)
                    && Math.abs(inputs.pivotVelocity.in(DegreesPerSecond))
                        < AlgaeIntakePivotConstants.kHomingVelocityThreshold.in(DegreesPerSecond)))
        .andThen(
            runOnce(
                () -> {
                  io.resetEncoder(AlgaeIntakePivotConstants.kPivotStartingAngle);
                  isHomed = true;
                }));
  }

  // sets voltage to the whole mechanism
  public Command setMechanismVoltage(Supplier<Voltage> volts) {
    return run(
        () -> {
          io.setPivotVoltage(volts.get());
        });
  }

  public Angle getAngle() {
    return inputs.pivotAngle;
  }

  public Angle getTargetAngle() {
    return targetAngle;
  }

  @Override
  public void periodic() { // updating inputs
    io.updateInputs(inputs);
  }

  public boolean atSetpoint() {
    return algaeIntakeClimbController.atSetpoint();
  }

  public boolean pivotIsHomed() {
    return isHomed;
  }

  public boolean inCollisionZone() {
    if (inputs.pivotAngle == null || inputs.pivotVelocity == null) return false;
    Angle effectiveAngle =
        inputs.pivotAngle.plus(
            inputs.pivotVelocity.times(
                RobotConstants.kRobotLoopPeriod.times(
                    2))); // angle after looking forward in time n loops based on current arm
    // velocity

    return effectiveAngle.compareTo(AlgaeIntakePivotConstants.kMinBlockedAngle) >= 0
        && effectiveAngle.compareTo(AlgaeIntakePivotConstants.kMaxBlockedAngle) <= 0;
  }
}
