/* (C) Robolancers 2025 */
package frc.robot.subsystems.algaeIntakeRollers;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.TunableConstant;
import java.util.function.Supplier;

/**
 * The same mechanism as algaeIntakeClimb but this controls the rollers instead of the pivot NOTE: +
 * voltage = intake
 */
@Logged
public class AlgaeIntakeRollers extends SubsystemBase {

  private AlgaeIntakeRollersIO io;
  private AlgaeIntakeRollersInputs inputs;

  private PIDController rollerController;
  private SimpleMotorFeedforward feedForward;

  private AlgaeIntakeRollersConfig config;

  public static AlgaeIntakeRollers create() {
    return RobotBase.isReal()
        ? new AlgaeIntakeRollers(
            new AlgaeIntakeRollersIOSpark(),
            AlgaeIntakeRollersIOSpark.config) // creates real mechanism if robot, sim if no robot,
        // ideal if disabled robot
        : new AlgaeIntakeRollers(new AlgaeIntakeRollersIOSim(), AlgaeIntakeRollersIOSim.config);
  }

  public static AlgaeIntakeRollers disable() {
    return new AlgaeIntakeRollers(
        new AlgaeIntakeRollersIOIdeal(), AlgaeIntakeRollersIOIdeal.config);
  }

  public AlgaeIntakeRollers(AlgaeIntakeRollersIO io, AlgaeIntakeRollersConfig config) {
    this.io = io;
    this.inputs = new AlgaeIntakeRollersInputs();
    this.config = config;

    rollerController = new PIDController(config.kP(), config.kI(), config.kD());
    feedForward = new SimpleMotorFeedforward(0, config.kV());
  }

  /**
   * Tune PID and feedforward constants(kP, kI, kD, kG) live on SmartDashboard so that we dont have
   * to re-run the code every time we change one of them.
   */
  public Command tune() {
    TunableConstant kP = new TunableConstant("/AlgaeIntakeRollers/kP", config.kP());
    TunableConstant kI = new TunableConstant("/AlgaeIntakeRollers/kI", config.kI());
    TunableConstant kD = new TunableConstant("/AlgaeIntakeRollers/kD", config.kD());
    TunableConstant kV = new TunableConstant("/AlgaeIntakeRollers/kV", config.kV());
    TunableConstant desiredAngularVelocity =
        new TunableConstant("/AlgaeIntakeRollers/DesiredVelocity", 0);

    return run(
        () -> {
          this.rollerController.setPID(kP.get(), kI.get(), kD.get());
          this.feedForward = new SimpleMotorFeedforward(0, kV.get());
          goToAngularVelocity(RPM.of(desiredAngularVelocity.get()));
        });
  }

  public void spinRollers(Voltage volts) {
    io.setRollerVoltage(volts);
  }

  public void goToAngularVelocity(AngularVelocity desiredAngularVelocity) {
    io.setRollerVoltage(
        Volts.of(
            rollerController.calculate(
                    inputs.rollerVelocity.in(RPM), desiredAngularVelocity.in(RPM))
                + feedForward.calculate(desiredAngularVelocity.in(RPM))));
  }

  public Command goToAngularVelocity(Supplier<AngularVelocity> angVel) {
    return run(() -> goToAngularVelocity(angVel.get()));
  }

  public Command
      intake() { // intakes algae until beam break breaks and registers algae in the mechanism
    return run(() -> spinRollers(AlgaeIntakeRollersConstants.kRollerIntakeVoltage));
  }

  public Command outtake() { // outtakes by spinning rollers outward
    return run(() -> spinRollers(AlgaeIntakeRollersConstants.kRollerOuttakeVoltage));
  }

  public Command setMechanismVoltage(Supplier<Voltage> volts) { // sets whole mechanism voltage
    return run(() -> spinRollers(volts.get()));
  }

  // stalls algae motor if algae is detected. Should be the default command
  public Command stallIfHasAlgae() {
    return run(
        () -> {
          if (inputs.hasAlgae) {
            spinRollers(AlgaeIntakeRollersConstants.kStallVoltage);
          } else {
            spinRollers(Volts.zero());
          }
        });
  }

  public boolean hasAlgae() {
    return inputs.hasAlgae;
  }

  @Override // updates inputs constatly
  public void periodic() {
    io.updateInputs(inputs);
  }
}
