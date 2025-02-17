/* (C) Robolancers 2025 */
package frc.robot.subsystems.drivetrain;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.util.MyAlliance;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/*
 * Drive interface. A real and simulator implementation is made out of this.
 * Using this, we can implement MapleSim.
 */
@Logged
public interface SwerveDrive extends Subsystem {

  // driveToPose PID controllers
  PIDController xPoseController =
      new PIDController(
          DrivetrainConstants.kTranslationGains.kP(),
          DrivetrainConstants.kTranslationGains.kI(),
          DrivetrainConstants.kTranslationGains.kD());

  PIDController yPoseController =
      new PIDController(
          DrivetrainConstants.kTranslationGains.kP(),
          DrivetrainConstants.kTranslationGains.kI(),
          DrivetrainConstants.kTranslationGains.kD());

  PIDController thetaController =
      new PIDController(
          DrivetrainConstants.kHeadingGains.kP(),
          DrivetrainConstants.kHeadingGains.kI(),
          DrivetrainConstants.kHeadingGains.kD());

  public static SwerveDrive create() {
    return RobotBase.isReal()
        ? new DrivetrainReal(
            TunerConstants.kTunerDrivetrain.getDriveTrainConstants(),
            TunerConstants.kTunerDrivetrain.getModuleConstants())
        : new DrivetrainSim();
  }

  public default void configureAutoBuilder() {
    try { // try and catch for config exception
      var config = RobotConfig.fromGUISettings();
      AutoBuilder.configure(
          this::getPose, // Supplier of current robot pose
          this::resetPose, // Consumer for seeding pose against auto
          this::getChassisSpeeds, // Supplier of current robot speeds
          // Consumer of ChassisSpeeds and feedforwards to drive the robot
          (speeds, feedforwards) ->
              driveRobotCentric(
                  speeds.vxMetersPerSecond,
                  speeds.vyMetersPerSecond,
                  speeds.omegaRadiansPerSecond,
                  feedforwards),
          new PPHolonomicDriveController(
              // PID constants for translation
              new PIDConstants(
                  DrivetrainConstants.kTranslationGains.kP(),
                  DrivetrainConstants.kTranslationGains.kI(),
                  DrivetrainConstants.kTranslationGains.kD()),
              // PID constants for rotation
              new PIDConstants(
                  DrivetrainConstants.kHeadingGains.kP(),
                  DrivetrainConstants.kHeadingGains.kI(),
                  DrivetrainConstants.kHeadingGains.kD())),
          config,
          // Assume the path needs to be flipped for Red vs Blue, this is normally the case
          () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
          this // Subsystem for requirements
          );
    } catch (Exception ex) {
      DriverStation.reportError(
          "Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
    }
  }

  default void configurePoseControllers() {
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  default ChassisSpeeds flipFieldSpeeds(ChassisSpeeds speeds) {
    return MyAlliance.isRed()
        ? new ChassisSpeeds(
            -speeds.vxMetersPerSecond, -speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond)
        : speeds;
  }

  default Rotation2d flipRotation(Rotation2d rotation) {
    return MyAlliance.isRed() ? rotation.plus(Rotation2d.k180deg) : rotation;
  }

  void setAlignmentSetpoint(Pose2d setpoint);

  /**
   * Checks whether the translation components and rotation are within 1e-9, the WPILib default
   * tolerance for equality
   *
   * @return whether the SwerveDrive is at the target alignment pose
   */
  boolean atPoseSetpoint();

  Command teleopDrive(
      DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotation);

  Command teleopDriveFixedHeading(
      DoubleSupplier translationX,
      DoubleSupplier translationY,
      DoubleSupplier rotationX,
      DoubleSupplier rotationY);

  Command driveFieldCentric(
      DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotation);

  Command driveRobotCentric(
      DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotation);

  // pathplanner chassis speeds consumer. Overload of driveRobotCentric
  void driveRobotCentric(
      double translationX, double translationY, double rotation, DriveFeedforwards feedforwards);

  void driveFixedHeading(double translationX, double translationY, Rotation2d rotation);

  // drive with heading controlled by PID
  default Command driveFixedHeading(
      DoubleSupplier translationX, DoubleSupplier translationY, Supplier<Rotation2d> rotation) {
    return run(
        () ->
            driveFixedHeading(
                translationX.getAsDouble(), translationY.getAsDouble(), rotation.get()));
  }

  // field relative auto drive w/ external pid controllers
  // drives to field pose for ONE loop
  void driveToFieldPose(Pose2d pose);

  // command wrapper for driving to field pose
  default Command driveToFieldPose(Supplier<Pose2d> pose) {
    return runOnce(
            () -> {
              xPoseController.reset();
              yPoseController.reset();
              thetaController.reset();
            })
        .andThen(run(() -> driveToFieldPose(pose.get())));
  }

  void resetPose(Pose2d pose);

  void setSwerveModuleStates(SwerveModuleState[] states);

  SwerveModuleState[] getMeasuredModuleStates();

  SwerveModulePosition[] getModulePositions();

  SwerveModuleState[] getTargetModuleStates();

  Pose2d getPose();

  Pose2d getReefVisionPose();

  ChassisSpeeds getChassisSpeeds();

  Rotation2d getHeading();

  void addVisionMeasurement(Pose2d visionRobotPose, double timeStampSeconds);

  void addVisionMeasurement(
      Pose2d visionRobotPose, double timeStampSeconds, Matrix<N3, N1> standardDeviations);

  void addReefVisionMeasurement(
      Pose2d visionRobotPose, double timeStampSeconds, Matrix<N3, N1> standardDeviations);
}
