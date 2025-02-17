/* (C) Robolancers 2025 */
package frc.robot.subsystems.algaeIntakePivot;

import edu.wpi.first.epilogue.Logged;

/* PID/Feedforward Configuration
kP = translation
kI = proportional to integral of error
kD = slows motion as object reaches target for greater accuracy
kG = counteracts gravity */
@Logged
public record AlgaeIntakePivotConfig(double kP, double kI, double kD, double kG) {}
