/* (C) Robolancers 2025 */
package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import org.photonvision.EstimatedRobotPose;

/*
 * TODO: note that if a class (like this one) is marked with @Logged and a custom logger is defined, the
 * auto-generated logger overrides but the custom logger is still imported, so build errors are caused
 * this may be a WPILib bug
 */
public record VisionEstimate(
    EstimatedRobotPose estimate,
    Matrix<N3, N1> stdDevs,
    String sourceName,
    CameraUsage sourceType) {}
