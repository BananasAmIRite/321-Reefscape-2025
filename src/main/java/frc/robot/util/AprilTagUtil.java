/* (C) Robolancers 2025 */
package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.RobotConstants;
import java.util.List;

public class AprilTagUtil {
  public static List<Pose2d> tagIDsToPoses(List<Integer> ids) {
    return ids.stream()
        .map(id -> RobotConstants.kAprilTagFieldLayout.getTagPose(id).get().toPose2d())
        .toList();
  }
}
