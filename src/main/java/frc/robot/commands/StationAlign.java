/* (C) Robolancers 2025 */
package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.SwerveDrive.AlignmentSetpoint;
import frc.robot.util.MyAlliance;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.DoubleSupplier;

public class StationAlign {

  public static final Map<Integer, Pose2d> stationPoses = new HashMap<>();

  private static final Distance kStationDistance = Inches.of(17.875);
  private static final Rotation2d kStationAlignmentRotation = Rotation2d.kZero;

  private static final List<Integer> blueStationTagIDs = List.of(12, 13);
  private static final List<Integer> redStationTagIDs = List.of(1, 2);

  private static final Distance kStationLeftAlignDistance = Meters.of(-0.52);
  private static final Distance kStationRightAlignDistance = Meters.of(0.52);

  private static final List<AprilTag> blueStationTags =
      RobotConstants.kAprilTagFieldLayout.getTags().stream()
          .filter(tag -> blueStationTagIDs.contains(tag.ID))
          .toList();
  private static final List<AprilTag> redStationTags =
      RobotConstants.kAprilTagFieldLayout.getTags().stream()
          .filter(tag -> redStationTagIDs.contains(tag.ID))
          .toList();

  /**
   * This method is run during Robot#autonomousInit() and Robot#teleopInit() to save computations,
   * only the station poses are loaded
   */
  public static void loadStationAlignmentPoses() {
    List<Integer> stationTagIds = MyAlliance.isRed() ? redStationTagIDs : blueStationTagIDs;
    for (Integer id : stationTagIds) {
      stationPoses.computeIfAbsent(id, StationAlign::getNearestCenterAlign);
    }
  }

  /**
   * Finds the pose of the nearest station tag
   *
   * @param robotPose the pose of the robot to find the nearest station tag pose for
   * @return null if robot alliance is unknown, otherwise a valid station tag pose
   */
  public static Pose2d getNearestStationPose(Pose2d robotPose) {
    // `Optional` means the Alliance may not exist yet, which must be handled to proceed
    Optional<Alliance> alliance = DriverStation.getAlliance();

    if (alliance.isEmpty()) return null;

    List<AprilTag> stationTags =
        alliance.get().equals(Alliance.Red) ? redStationTags : blueStationTags;

    return robotPose.nearest(stationTags.stream().map(tag -> tag.pose.toPose2d()).toList());
  }

  /**
   * Finds the ID of the nearest station tag on the alliance side
   *
   * @param robotPose the pose of the robot to find the nearest station tag ID for
   * @return -1 if robot alliance is unknown, otherwise a valid station tag ID
   */
  public static int getNearestStationID(Pose2d robotPose) {
    Pose2d nearest = getNearestStationPose(robotPose);

    // handle the null that getNearestStationPose() may return when robot alliance is unknown
    if (nearest == null) return -1;

    return RobotConstants.kAprilTagFieldLayout.getTags().stream()
        .filter(tag -> tag.pose.toPose2d().equals(nearest))
        .toList()
        .get(0)
        .ID;
  }

  /**
   * Intended for aligning to the station
   *
   * @param stationTagID a valid station tag ID
   * @return null if no station tag of the ID specified is found, otherwise a valid robot pose
   *     aligned with the center of the nearest station tag
   */
  private static Pose2d getNearestCenterAlign(int stationTagID) {
    // `Optional` here means there may not be a tag with the specified ID, again must be handled
    Optional<Pose3d> tagPose = RobotConstants.kAprilTagFieldLayout.getTagPose(stationTagID);

    if (tagPose.isEmpty()) return null;

    Pose2d aprilTagPose = tagPose.get().toPose2d();

    Pose2d resultPose =
        aprilTagPose.plus(
            new Transform2d(kStationDistance, Meter.zero(), kStationAlignmentRotation));

    return resultPose;
  }

  private static Pose2d getNearestRightAlign(int stationTagID) {
    // `Optional` here means there may not be a tag with the specified ID, again must be handled
    Optional<Pose3d> tagPose = RobotConstants.kAprilTagFieldLayout.getTagPose(stationTagID);

    if (tagPose.isEmpty()) return null;

    Pose2d aprilTagPose = tagPose.get().toPose2d();

    Pose2d resultPose =
        aprilTagPose.plus(
            new Transform2d(
                kStationDistance, kStationRightAlignDistance, kStationAlignmentRotation));

    return resultPose;
  }

  private static Pose2d getNearestLeftAlign(int stationTagID) {
    // `Optional` here means there may not be a tag with the specified ID, again must be handled
    Optional<Pose3d> tagPose = RobotConstants.kAprilTagFieldLayout.getTagPose(stationTagID);

    if (tagPose.isEmpty()) return null;

    Pose2d aprilTagPose = tagPose.get().toPose2d();

    Pose2d resultPose =
        aprilTagPose.plus(
            new Transform2d(
                kStationDistance, kStationLeftAlignDistance, kStationAlignmentRotation));

    return resultPose;
  }

  /** Drives to align against the center of the nearest station, no manual driving */
  public static Command goToNearestCenterAlign(SwerveDrive swerveDrive) {
    return swerveDrive.driveToFieldPose(
        () ->
            new AlignmentSetpoint(
                getNearestCenterAlign(getNearestStationID(swerveDrive.getPose())), true),
        swerveDrive::getPose);
  }

  public static Command goToNearestRightAlign(SwerveDrive swerveDrive) {
    return swerveDrive.driveToFieldPose(
        () ->
            new AlignmentSetpoint(
                getNearestRightAlign(getNearestStationID(swerveDrive.getPose())), true),
        swerveDrive::getPose);
  }

  public static Command goToNearestLeftAlign(SwerveDrive swerveDrive) {
    return swerveDrive.driveToFieldPose(
        () ->
            new AlignmentSetpoint(
                getNearestLeftAlign(getNearestStationID(swerveDrive.getPose())), true),
        swerveDrive::getPose);
  }

  /** Maintain translational driving while rotating toward the nearest station tag */
  public static Command rotateToNearestStationTag(
      SwerveDrive swerveDrive, DoubleSupplier x, DoubleSupplier y) {
    return swerveDrive.driveFixedHeading(
        x,
        y,
        () ->
            getNearestStationPose(swerveDrive.getPose())
                .getRotation()
                .plus(kStationAlignmentRotation));
  }

  public static double getStationDistance(SwerveDrive swerveDrive) {
    return swerveDrive
        .getPose()
        .getTranslation()
        .getDistance(getNearestStationPose(swerveDrive.getPose()).getTranslation());
  }
}
