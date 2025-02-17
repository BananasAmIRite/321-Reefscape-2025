/* (C) Robolancers 2025 */
package frc.robot.subsystems.elevator;

import edu.wpi.first.epilogue.Logged;

// Adjustable values, created with every arm instance
@Logged
public record ElevatorConfig(
    double kP, double kI, double kD, double kG, double kS, double kV, double kA) {}
