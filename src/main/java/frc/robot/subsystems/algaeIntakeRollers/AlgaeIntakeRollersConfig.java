/* (C) Robolancers 2025 */
package frc.robot.subsystems.algaeIntakeRollers;

import edu.wpi.first.epilogue.Logged;

/**
 * PID/Feedforward Configurations: kP = translation kI = proportional to integral of error kD =
 * slows motion as object reaches target for greater accuracy kV = velocity
 */
@Logged
public record AlgaeIntakeRollersConfig(double kP, double kI, double kD, double kV) {}
