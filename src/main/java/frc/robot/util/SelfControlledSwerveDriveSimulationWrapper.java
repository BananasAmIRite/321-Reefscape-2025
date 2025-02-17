/* (C) Robolancers 2025 */
package frc.robot.util;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

public class SelfControlledSwerveDriveSimulationWrapper
    extends SelfControlledSwerveDriveSimulation {
  // stolen from maple sim drive config
  SwerveDriveKinematics swerveKinematics =
      new SwerveDriveKinematics(
          new Translation2d(
              DrivetrainConstants.kTrackWidth.in(Meters) / 2,
              DrivetrainConstants.kWheelBase.in(Meters) / 2),
          new Translation2d(
              DrivetrainConstants.kTrackWidth.in(Meters) / 2,
              -DrivetrainConstants.kWheelBase.in(Meters) / 2),
          new Translation2d(
              -DrivetrainConstants.kTrackWidth.in(Meters) / 2,
              DrivetrainConstants.kWheelBase.in(Meters) / 2),
          new Translation2d(
              -DrivetrainConstants.kTrackWidth.in(Meters) / 2,
              -DrivetrainConstants.kWheelBase.in(Meters) / 2));

  public SelfControlledSwerveDriveSimulationWrapper(SwerveDriveSimulation swerveDriveSimulation) {
    super(swerveDriveSimulation);
  }

  public SelfControlledSwerveDriveSimulationWrapper(
      SwerveDriveSimulation swerveDriveSimulation,
      Matrix<N3, N1> stateStdDevs,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    super(swerveDriveSimulation, stateStdDevs, visionMeasurementStdDevs);
  }

  @Override
  public void runChassisSpeeds(
      ChassisSpeeds chassisSpeeds,
      Translation2d centerOfRotationMeters,
      boolean fieldCentricDrive,
      boolean discretizeSpeeds) {
    SwerveModuleState[] states = swerveKinematics.toSwerveModuleStates(chassisSpeeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(states, DrivetrainConstants.kMaxLinearVelocity);

    ChassisSpeeds desatSpeeds = swerveKinematics.toChassisSpeeds(states);

    super.runChassisSpeeds(
        desatSpeeds, centerOfRotationMeters, fieldCentricDrive, discretizeSpeeds);
  }

  public SwerveDriveKinematics getKinematics() {
    return swerveKinematics;
  }
}
