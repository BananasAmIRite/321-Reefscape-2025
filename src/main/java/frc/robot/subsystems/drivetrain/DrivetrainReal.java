/* (C) Robolancers 2025 */
package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.ModuleRequest;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants;
import java.util.function.DoubleSupplier;

/*
 * Real Drivetrain using CTRE SwerveDrivetrain and SwerveRequests
 */

@Logged
public class DrivetrainReal extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder>
    implements SwerveDrive {
  private final SwerveRequest.FieldCentric fieldCentricRequest =
      new SwerveRequest.FieldCentric()
          .withDriveRequestType(DriveRequestType.Velocity)
          .withDesaturateWheelSpeeds(true);

  private final SwerveRequest.ApplyRobotSpeeds robotCentricRequest =
      new SwerveRequest.ApplyRobotSpeeds()
          .withDriveRequestType(DriveRequestType.Velocity)
          .withDesaturateWheelSpeeds(true);

  private final SwerveRequest.FieldCentricFacingAngle fieldCentricFacingAngleRequest =
      new SwerveRequest.FieldCentricFacingAngle()
          .withDriveRequestType(DriveRequestType.Velocity)
          .withDesaturateWheelSpeeds(true);

  private final SwerveDrivePoseEstimator reefPoseEstimator;

  private Pose2d alignmentSetpoint = Pose2d.kZero;

  public DrivetrainReal(
      SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
    // create CTRE Swervedrivetrain
    super(TalonFX::new, TalonFX::new, CANcoder::new, drivetrainConstants, modules);
    configNeutralMode(NeutralModeValue.Brake);
    configureAutoBuilder();
    configurePoseControllers();

    this.reefPoseEstimator =
        new SwerveDrivePoseEstimator(
            getKinematics(), getHeading(), getModulePositions(), getPose());
  }

  @Override
  public Command teleopDrive(
      DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotation) {
    return run(
        () -> {
          var speeds =
              ChassisSpeeds.discretize(
                  translationX.getAsDouble(),
                  translationY.getAsDouble(),
                  rotation.getAsDouble(),
                  RobotConstants.kRobotLoopPeriod.in(Seconds));

          // x braking
          // if(Math.abs(newTranslationX) < DriveConstants.kDriveDeadband &&
          // Math.abs(newTranslationY) < DriveConstants.kDriveDeadband &&
          // Math.abs(newRotation) < DriveConstants.kRotationDeadband){
          // setControl(new SwerveRequest.SwerveDriveBrake())};

          setControl(
              fieldCentricRequest
                  .withVelocityX(speeds.vxMetersPerSecond)
                  .withVelocityY(speeds.vyMetersPerSecond)
                  .withRotationalRate(speeds.omegaRadiansPerSecond)
                  .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective));
        });
  }

  @Override
  public Command teleopDriveFixedHeading(
      DoubleSupplier translationX,
      DoubleSupplier translationY,
      DoubleSupplier rotationX,
      DoubleSupplier rotationY) {
    return run(
        () -> {
          Rotation2d desiredRotation =
              flipRotation(
                  Rotation2d.fromRadians(
                      Math.atan2(rotationX.getAsDouble(), rotationY.getAsDouble())));

          var speeds =
              ChassisSpeeds.discretize(
                  translationX.getAsDouble(),
                  translationY.getAsDouble(),
                  0,
                  RobotConstants.kRobotLoopPeriod.in(Seconds));

          setControl(
              fieldCentricFacingAngleRequest
                  .withDriveRequestType(DriveRequestType.Velocity)
                  .withVelocityX(speeds.vxMetersPerSecond)
                  .withVelocityY(speeds.vyMetersPerSecond)
                  .withTargetDirection(desiredRotation)
                  .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective));
        });
  }

  @Override
  public Command driveFieldCentric(
      DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotation) {
    return run(
        () -> {
          var speeds =
              ChassisSpeeds.discretize(
                  translationX.getAsDouble(),
                  translationY.getAsDouble(),
                  rotation.getAsDouble(),
                  RobotConstants.kRobotLoopPeriod.in(Seconds));

          setControl(
              fieldCentricRequest
                  .withVelocityX(speeds.vxMetersPerSecond)
                  .withVelocityY(speeds.vyMetersPerSecond)
                  .withRotationalRate(speeds.omegaRadiansPerSecond));
        });
  }
  ;

  @Override
  public Command driveRobotCentric(
      DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotation) {
    return run(
        () ->
            driveRobotCentric(
                translationX.getAsDouble(),
                translationY.getAsDouble(),
                rotation.getAsDouble(),
                DriveFeedforwards.zeros(4)));
  }

  @Override
  public void driveRobotCentric(
      double translationX, double translationY, double rotation, DriveFeedforwards feedforwards) {
    // var speeds =
    //     ChassisSpeeds.discretize(
    //         translationX, translationY, rotation, RobotConstants.kRobotLoopPeriod.in(Seconds));

    var speeds = new ChassisSpeeds(translationX, translationY, rotation);

    setControl(
        robotCentricRequest
            .withSpeeds(speeds)
            .withDriveRequestType(DriveRequestType.Velocity)
            .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
            .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons()));
  }

  @Override
  public void driveToFieldPose(Pose2d pose) {
    var targetSpeeds =
        ChassisSpeeds.discretize(
            xPoseController.calculate(getPose().getX(), pose.getX())
                * DrivetrainConstants.kMaxLinearVelocity.in(MetersPerSecond),
            yPoseController.calculate(getPose().getY(), pose.getY())
                * DrivetrainConstants.kMaxLinearVelocity.in(MetersPerSecond),
            thetaController.calculate(
                    getPose().getRotation().getRadians(), pose.getRotation().getRadians())
                * DrivetrainConstants.kMaxAngularVelocity.in(RadiansPerSecond),
            RobotConstants.kRobotLoopPeriod.in(Seconds));

    setControl(
        fieldCentricRequest
            .withDriveRequestType(DriveRequestType.Velocity)
            .withVelocityX(targetSpeeds.vxMetersPerSecond)
            .withVelocityY(targetSpeeds.vyMetersPerSecond)
            .withRotationalRate(targetSpeeds.omegaRadiansPerSecond));
  }

  // drive with absolute heading control
  @Override
  public void driveFixedHeading(double translationX, double translationY, Rotation2d rotation) {
    var speeds =
        ChassisSpeeds.discretize(
            translationX, translationY, 0, RobotConstants.kRobotLoopPeriod.in(Seconds));

    setControl(
        fieldCentricFacingAngleRequest
            .withDriveRequestType(DriveRequestType.Velocity)
            .withVelocityX(speeds.vxMetersPerSecond)
            .withVelocityY(speeds.vyMetersPerSecond)
            .withTargetDirection(rotation)
            .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance));
  }

  @Override
  public void setAlignmentSetpoint(Pose2d setpoint) {
    alignmentSetpoint = setpoint;
  }

  @Override
  public boolean atPoseSetpoint() {
    final var currentPose = getPose();
    return currentPose.getTranslation().getDistance(alignmentSetpoint.getTranslation())
            < DrivetrainConstants.kAlignmentSetpointTranslationTolerance.in(Meters)
        && Math.abs(currentPose.getRotation().minus(alignmentSetpoint.getRotation()).getDegrees())
            < DrivetrainConstants.kAlignmentSetpointRotationTolerance.in(Degrees);
  }

  @Override
  public void setSwerveModuleStates(SwerveModuleState[] states) {
    for (int i = 0; i < super.getModules().length; i++) {
      super.getModule(i).apply(new ModuleRequest().withState(states[i]));
    }
  }

  @Logged(name = "MeasuredModuleStates")
  @Override
  public SwerveModuleState[] getMeasuredModuleStates() {
    return super.getState().ModuleStates;
  }

  @Logged(name = "MeasuredModulePositions")
  @Override
  public SwerveModulePosition[] getModulePositions() {
    return super.getState().ModulePositions;
  }

  @Logged(name = "TargetModuleStates")
  @Override
  public SwerveModuleState[] getTargetModuleStates() {
    return super.getState().ModuleTargets;
  }

  @Logged(name = "ReefVisionEstimatedPose")
  @Override
  public Pose2d getReefVisionPose() {
    return reefPoseEstimator.getEstimatedPosition();
  }

  @Logged(name = "MeasuredRobotPose")
  @Override
  public Pose2d getPose() {
    return super.getState().Pose;
  }

  @Logged(name = "MeasuredRobotRelativeChassisSpeeds")
  @Override
  public ChassisSpeeds getChassisSpeeds() {
    return super.getState().Speeds;
  }

  @Logged(name = "MeasuredHeadingRad")
  @Override
  public Rotation2d getHeading() {
    return new Rotation2d(super.getPigeon2().getYaw().getValue().in(Radians));
  }

  @Override
  public void addReefVisionMeasurement(
      Pose2d visionRobotPose, double timeStampSeconds, Matrix<N3, N1> standardDeviations) {
    reefPoseEstimator.addVisionMeasurement(visionRobotPose, timeStampSeconds, standardDeviations);
  }

  @Override
  public void periodic() {
    reefPoseEstimator.update(getHeading(), getModulePositions());
  }
}
