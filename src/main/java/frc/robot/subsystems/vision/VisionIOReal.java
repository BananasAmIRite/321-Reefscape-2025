/* (C) Robolancers 2025 */
package frc.robot.subsystems.vision;

import edu.wpi.first.epilogue.Logged;
import frc.robot.subsystems.vision.VisionConstants.CameraConfig;
import java.util.List;
import java.util.Objects;
import java.util.stream.Stream;
import org.photonvision.PhotonCamera;

@Logged
public class VisionIOReal implements VisionIO {
  private final List<Camera> cameras;

  public VisionIOReal(CameraConfig... configs) {
    cameras =
        Stream.of(configs)
            .map(config -> new Camera(config, new PhotonCamera(config.cameraName())))
            .toList();
  }

  @Override
  public VisionEstimate[] getLatestEstimates() {
    return cameras.stream()
        .map(Camera::tryLatestEstimate)
        .filter(Objects::nonNull)
        .toArray(VisionEstimate[]::new);
  }
}
