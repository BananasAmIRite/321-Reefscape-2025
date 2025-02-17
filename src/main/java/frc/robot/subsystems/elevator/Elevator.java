/* (C) Robolancers 2025 */
package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.TunableConstant;
import java.util.function.Supplier;

@Logged
public class Elevator extends SubsystemBase {
  // Initialize all parts of Elevator Subsystem, as well as pid controller
  private ElevatorIO io;
  private ElevatorInputs inputs;

  private Distance targetHeight = ElevatorConstants.kElevatorStartingHeight;

  private boolean isHomed = false;

  /* Method that creates the Elevator object as the real/sim io by checking if we're running
  a sim or not */
  public static Elevator create() {
    return RobotBase.isReal()
        ? new Elevator(new ElevatorIOTalon())
        : new Elevator(new ElevatorIOSim());
  }

  public static Elevator disable() {
    return new Elevator(new ElevatorIOIdeal());
  }

  // Eleavtor Constructor
  // Creates Elevator, sets initialized variables to the real/sim values from create() method above
  public Elevator(ElevatorIO io) {
    this.io = io;
    this.inputs = new ElevatorInputs();

    // set position to starting position
    io.resetEncoderPosition();
  }

  /* Below are methods & their commands for simple robot operations
  Because we need commands for when buttons are pressed, we create methods first then use
  run(()->{method}) */

  // Sets voltage
  public void setVoltage(Voltage volts) {
    io.setVoltage(volts);
  }

  public Command setVoltage(Supplier<Voltage> volts) {
    return run(
        () -> {
          setVoltage(volts.get());
        });
  }

  // Goes to height
  public void goToHeight(Distance targetHeight) {
    this.targetHeight = targetHeight;
    io.goToPosition(targetHeight);
  }

  // returns a Command to go to height
  public Command goToHeight(Supplier<Distance> targetHeight) {
    return run(
        () -> {
          double setpoint =
              MathUtil.clamp(
                  targetHeight.get().in(Meters),
                  ElevatorConstants.kElevatorMinimumHeight.in(Meters),
                  ElevatorConstants.kElevatorMaximumHeight.in(Meters));
          goToHeight(Meters.of(setpoint));
        });
  }

  /* Command to "home" the encoder (go to starting position & set encoder to said position)
  - Sets voltage to a constant negative voltage
  - Once current spikes (signaling motor running into resistance) & the V ~0, encoder speed is
    about zero
  - Set to 0 */
  public Command homeEncoder() {
    return setVoltage(() -> ElevatorConstants.kHomingVoltage)
        .until(
            () ->
                (inputs.current.in(Amp) > ElevatorConstants.kHomingCurrentThreshold.in(Amp)
                    && Math.abs(inputs.velocity.in(MetersPerSecond))
                        < ElevatorConstants.kHomingVelocityThreshold.in(MetersPerSecond)))
        .andThen(
            runOnce(
                () -> {
                  io.resetEncoderPosition();
                  isHomed = true;
                }));
  }

  /* When the command runs, tunable constants are created & PID controller values & target height are
  // set to said tunable values */
  public Command tune() {
    TunableConstant kP = new TunableConstant("/Elevator/kP", 0);
    TunableConstant kI = new TunableConstant("/Elevator/kI", 0);
    TunableConstant kD = new TunableConstant("/Elevator/kD", 0);
    TunableConstant kG = new TunableConstant("/Elevator/kG", 0);
    TunableConstant kS = new TunableConstant("/Elevator/kS", 0);
    TunableConstant kV = new TunableConstant("/Elevator/kV", 0);
    TunableConstant kA = new TunableConstant("/Elevator/kA", 0);
    TunableConstant targetHeight = new TunableConstant("/Elevator/targetHeight", 0);

    return runOnce(
            () -> {
              io.setOnboardPID(
                  new ElevatorConfig(
                      kP.get(), kI.get(), kD.get(), kG.get(), kS.get(), kV.get(), kA.get()));
            })
        .andThen(
            run(
                () -> {
                  goToHeight(Meters.of(targetHeight.get()));
                }));
  }

  // Loops repeatedly
  public void periodic() {
    // Constantly updates inputs
    io.updateInputs(inputs);
  }

  public Distance getHeight() {
    return inputs.height;
  }

  public Distance getTargetHeight() {
    return targetHeight;
  }

  public boolean elevatorIsHomed() {
    return isHomed;
  }

  public boolean inCollisionZone() {
    if (getHeight() == null) return false;
    return getHeight().compareTo(ElevatorConstants.kElevatorDangerHeight) < 0;
  }

  public boolean atSetpoint() {
    return inputs.atSetpoint;
  }
}
