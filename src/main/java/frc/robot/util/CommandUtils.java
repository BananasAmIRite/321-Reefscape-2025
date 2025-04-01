/* (C) Robolancers 2025 */
package frc.robot.util;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;

public class CommandUtils {
  public static Command dynamicEither(Command onTrue, Command onFalse, BooleanSupplier value) {
    return onFalse.until(value).andThen(onTrue.onlyWhile(value));
  }
}
