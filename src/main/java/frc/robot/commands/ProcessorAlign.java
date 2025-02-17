/* (C) Robolancers 2025 */
package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;

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
import frc.robot.util.MyAlliance;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.DoubleSupplier;

public class ProcessorAlign {

  public static final Map<Integer, Pose2d> processorPoses = new HashMap<>();

  private static final Distance kProcessorDistance = Inches.of(14);
  private static final Rotation2d kProcessorAlignmentRotation = Rotation2d.kCCW_90deg;

  private static final Transform2d kProcessorAlignTransform =
      new Transform2d(kProcessorDistance, Meter.zero(), kProcessorAlignmentRotation);

  private static final int kBlueProcessorTagID = 16;
  private static final int kRedProcessorTagID = 3;

  // TODO: use units
  public static final Pose2d kBlueProcessorPose = new Pose2d(5.983, 0.395, Rotation2d.kZero);
  public static final Pose2d kRedProcessorPose = new Pose2d(11.437, 7.675, Rotation2d.kZero);

  private static final Pose2d kBlueProcessorTag =
      RobotConstants.kAprilTagFieldLayout.getTagPose(kBlueProcessorTagID).get().toPose2d();
  private static final Pose2d kRedProcessorTag =
      RobotConstants.kAprilTagFieldLayout.getTagPose(kRedProcessorTagID).get().toPose2d();

  public static final Distance kAlignmentDeadbandRange = Meters.of(0.75);

  /**
   * This method is run when DriverStation connects to save computations mid-match, only the
   * processor poses are loaded
   */
  public static void loadProcessorAlignmentPoses() {
    processorPoses.computeIfAbsent(kBlueProcessorTagID, ProcessorAlign::getNearestAlign);
    processorPoses.computeIfAbsent(kRedProcessorTagID, ProcessorAlign::getNearestAlign);
  }

  /**
   * Finds the pose of the nearest processor tag
   *
   * @param robotPose the pose of the robot to find the nearest processor tag pose for
   * @return null if robot alliance is unknown, otherwise a valid processor tag pose
   */
  public static Pose2d getNearestProcessorPose(Pose2d robotPose) {
    // `Optional` means the Alliance may not exist yet, which must be handled to proceed
    Optional<Alliance> alliance = DriverStation.getAlliance();

    if (alliance.isEmpty()) return null;

    List<Pose2d> processorTag =
        List.of(alliance.get().equals(Alliance.Red) ? kRedProcessorTag : kBlueProcessorTag);

    return robotPose.nearest(processorTag);
  }

  /**
   * Finds the ID of the nearest processor tag on the alliance side
   *
   * @param robotPose the pose of the robot to find the nearest processor tag ID for
   * @return -1 if robot alliance is unknown, otherwise a valid processor tag ID
   */
  public static int getNearestProcessorID(Pose2d robotPose) {
    Pose2d nearest = getNearestProcessorPose(robotPose);

    // handle the null that getNearestProcessorPose() may return when robot alliance is unknown
    if (nearest == null) return -1;

    return RobotConstants.kAprilTagFieldLayout.getTags().stream()
        .filter(tag -> tag.pose.toPose2d().equals(nearest))
        .toList()
        .get(0)
        .ID;
  }

  /**
   * Intended for aligning to the processor
   *
   * @param processorTagID a valid processor tag ID
   * @return null if no processor tag of the ID specified is found, otherwise a valid robot pose
   *     aligned with the center of the nearest processor tag
   */
  private static Pose2d getNearestAlign(int processorTagID) {
    // `Optional` here means there may not be a tag with the specified ID, again must be handled
    Optional<Pose3d> tagPose = RobotConstants.kAprilTagFieldLayout.getTagPose(processorTagID);

    if (tagPose.isEmpty()) return null;

    Pose2d aprilTagPose = tagPose.get().toPose2d();

    Pose2d resultPose = aprilTagPose.plus(kProcessorAlignTransform);

    return resultPose;
  }

  /** Drives to align against the center of the nearest processor, no manual driving */
  public static Command goToNearestAlign(SwerveDrive swerveDrive) {
    return swerveDrive.driveToFieldPose(
        () -> {
          final Pose2d target = processorPoses.get(getNearestProcessorID(swerveDrive.getPose()));
          swerveDrive.setAlignmentSetpoint(target);
          return target;
        });
  }

  /** Maintain translational driving while rotating toward the nearest processor tag */
  public static Command rotateToNearestProcessor(
      SwerveDrive swerveDrive, DoubleSupplier x, DoubleSupplier y) {
    return swerveDrive.driveFixedHeading(
        x,
        y,
        () ->
            getNearestProcessorPose(swerveDrive.getPose())
                .getRotation()
                .plus(kProcessorAlignmentRotation));
  }

  // if robot is within 2 meters of either red or blue processor, auto-align will NOT work
  public static boolean isWithinProcessorRange(SwerveDrive drive, Distance deadband) {
    Pose2d centerPos = MyAlliance.isRed() ? kRedProcessorPose : kBlueProcessorPose;
    double deadbandDistance =
        Math.hypot(
            drive.getPose().getX() - centerPos.getX(), drive.getPose().getY() - centerPos.getY());

    return deadbandDistance < deadband.in(Meters);
  }
}
