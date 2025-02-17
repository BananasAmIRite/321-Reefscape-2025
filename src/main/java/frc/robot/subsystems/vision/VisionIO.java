/* (C) Robolancers 2025 */
package frc.robot.subsystems.vision;

import edu.wpi.first.epilogue.Logged;

@Logged
public interface VisionIO {
  VisionEstimate[] getLatestEstimates();
}
