/* (C) Robolancers 2025 */
package frc.robot.subsystems.leds;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Comparator;
import java.util.TreeSet;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

@Logged
public class Leds extends SubsystemBase {
  // priority (greater # means more important), condition for pattern to apply, pattern to apply
  public record Signal(int priority, BooleanSupplier condition, Supplier<LEDPattern> pattern) {}

  private static Leds instance = new Leds();

  public static Leds getInstance() {
    return instance;
  }

  // LED
  public final AddressableLED strip;
  private final AddressableLEDBuffer buffer;
  private final AddressableLEDBufferView leftBuffer;
  private final AddressableLEDBufferView rightBuffer;
  private static TreeSet<Signal> signals;
  private LEDPattern currentPattern = LedsConstants.kDefault;

  public boolean isRotateAligning = false;
  public boolean isReefAligning = false;
  public boolean isIntaking = false;
  public boolean isOuttaking = false;
  public boolean isTagAligning = false;

  public Leds() {
    this.strip = new AddressableLED(LedsConstants.kPort);
    this.buffer = new AddressableLEDBuffer(LedsConstants.kLength);
    this.leftBuffer = buffer.createView(0, LedsConstants.kLength / 2);
    this.rightBuffer =
        buffer.createView(LedsConstants.kLength / 2, LedsConstants.kLength - 1).reversed();
    this.strip.setLength(buffer.getLength());
    this.strip.setData(buffer);
    this.strip.start();

    // sorted in descending order of priority
    signals = new TreeSet<Signal>(Comparator.comparingInt(Signal::priority).reversed());
  }

  // register signals into the tree set. If priority is already taken, don't add it
  public void registerSignal(
      int priority, BooleanSupplier condition, Supplier<LEDPattern> pattern) {
    final var priorityIsAlreadyClaimed =
        signals.stream().mapToInt(Signal::priority).anyMatch(p -> p == priority);

    if (priorityIsAlreadyClaimed) {
      DriverStation.reportWarning("Priority " + priority + " is already claimed", true);
      return;
    }

    signals.add(new Signal(priority, condition, pattern));
  }

  // default command to turn off leds
  public Command off() {
    return run(
        () -> {
          LEDPattern.kOff.applyTo(buffer);
          strip.setData(buffer);
        });
  }

  // default command to turn on & update leds
  public Command updateLeds() {
    return run(
        () -> {
          // set new pattern to the pattern of the most important signal that is true
          for (var signal : signals) {
            if (signal.condition.getAsBoolean()) {
              currentPattern = signal.pattern.get();
              break;
            }
          }

          // apply the pattern to the led strip
          currentPattern.applyTo(leftBuffer);
          currentPattern.applyTo(rightBuffer);
          strip.setData(buffer);
        });
  }

  public double calculateProgressBar(
      Distance elevatorCurrentHeight,
      Distance elevatorSetpoint,
      Angle armCurrentAngle,
      Angle armSetpoint,
      Pose2d driveCurrentPose,
      Pose2d driveSetpointPose) {

    double elevatorError =
        calculateError(elevatorCurrentHeight.in(Meters), elevatorSetpoint.in(Meters));
    double armError = calculateError(armCurrentAngle.in(Radians), armSetpoint.in(Radians));
    double driveTranslationError =
        driveCurrentPose.getTranslation().getDistance(driveSetpointPose.getTranslation());
    double driveRotationError =
        driveCurrentPose.getRotation().minus(driveSetpointPose.getRotation()).getRadians();

    return 1 / (1 + elevatorError + armError + driveTranslationError + driveRotationError);
  }

  private double calculateError(double currentMeasurement, double setpoint) {
    return Math.abs(currentMeasurement - setpoint);
  }
}
