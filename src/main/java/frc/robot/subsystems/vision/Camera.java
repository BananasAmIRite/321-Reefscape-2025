/* (C) Robolancers 2025 */
package frc.robot.subsystems.vision;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.RobotConstants;
import frc.robot.subsystems.vision.VisionConstants.CameraConfig;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

@Logged
public class Camera {
  // for logging
  private final String name;

  private final CameraUsage usage;

  private final PhotonCamera camera;

  private final PhotonPoseEstimator poseEstimator;

  // for logging
  private VisionEstimate latestValidEstimate;

  public Camera(CameraConfig config, PhotonCamera camera) {
    this.name = config.cameraName();

    this.usage = config.usage();

    this.camera = camera;

    this.poseEstimator =
        new PhotonPoseEstimator(
            RobotConstants.kAprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            config.robotToCamera());
  }

  @NotLogged
  public VisionEstimate tryLatestEstimate() {
    if (!camera.isConnected()) return null;

    final var unreadResults = camera.getAllUnreadResults();

    if (unreadResults.isEmpty()) return null;

    final var latestResult = unreadResults.get(unreadResults.size() - 1);

    if (!latestResult.hasTargets()) return null;

    final var estimate = poseEstimator.update(latestResult);

    return estimate
        .filter(
            poseEst ->
                VisionConstants.kAllowedFieldArea.contains(
                    poseEst.estimatedPose.getTranslation().toTranslation2d()))
        .map(
            photonEst -> {
              final var visionEst =
                  new VisionEstimate(photonEst, calculateStdDevs(photonEst), name, usage);
              latestValidEstimate = visionEst;
              return visionEst;
            })
        .orElse(null);
  }

  // could be absolute nonsense, open to tuning constants for each robot camera config
  // assumes `result` has targets
  @NotLogged
  private Matrix<N3, N1> calculateStdDevs(EstimatedRobotPose visionPoseEstimate) {
    // weighted average by ambiguity
    final double avgTargetDistance =
        visionPoseEstimate.targetsUsed.stream()
                .mapToDouble(
                    t -> {
                      final double bestCameraToTargetDistance =
                          t.getBestCameraToTarget()
                              .getTranslation()
                              .getDistance(new Translation3d());

                      return bestCameraToTargetDistance;
                    })
                .reduce(0.0, Double::sum)
            / (visionPoseEstimate.targetsUsed.size());

    final double translationStdDev =
        VisionConstants.kTranslationStdDevCoeff
            * Math.pow(avgTargetDistance, 3)
            / Math.pow(visionPoseEstimate.targetsUsed.size(), 3);
    final double rotationStdDev =
        VisionConstants.kRotationStdDevCoeff
            * Math.pow(avgTargetDistance, 3)
            / Math.pow(visionPoseEstimate.targetsUsed.size(), 3);

    return VecBuilder.fill(translationStdDev, translationStdDev, rotationStdDev);
  }
}
