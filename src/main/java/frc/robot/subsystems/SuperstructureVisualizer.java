/* (C) Robolancers 2025 */
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.algaeIntakePivot.AlgaeIntakePivotConstants;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevatorarm.ElevatorArmConstants;
import frc.robot.util.VirtualSubsystem;
import java.util.function.Supplier;

@Logged
public class SuperstructureVisualizer extends VirtualSubsystem {

  private Mechanism2d mechanism;
  private MechanismLigament2d elevator;
  private MechanismLigament2d arm;

  private Supplier<Distance> elevatorSetpoint = () -> ElevatorConstants.kElevatorMinimumHeight;
  private Supplier<Angle> armSetpoint = () -> ElevatorArmConstants.kStartAngle;
  private Supplier<Angle> algaePivotSetpoint = () -> AlgaeIntakePivotConstants.kPivotStartingAngle;

  private Pose3d elevatorFirstStagePose = new Pose3d();
  private Pose3d elevatorSecondStagePose = new Pose3d();
  private Pose3d shoulderPose = new Pose3d();
  private Pose3d elbowPose = new Pose3d();
  private Pose3d algaeIntakePose = new Pose3d();

  /* Constructor that establishes the characteristics of our robot superstructure
   */

  public SuperstructureVisualizer(
      Supplier<Distance> elevatorSetpoint,
      Supplier<Angle> armSetpoint,
      Supplier<Angle> algaeIntakeSetpoint) {
    this.elevatorSetpoint = elevatorSetpoint;
    this.armSetpoint = armSetpoint;
    this.algaePivotSetpoint = algaeIntakeSetpoint;

    this.mechanism = new Mechanism2d(Inches.of(29).in(Meters), Inches.of(80).in(Meters));

    MechanismRoot2d root =
        mechanism.getRoot(
            "Root",
            VisualizerConstants.elevatorRoot2d.getX(),
            VisualizerConstants.elevatorRoot2d.getY());

    this.elevator =
        root.append(
            new MechanismLigament2d(
                "Elevator",
                ElevatorConstants.kElevatorStartingHeight.in(Meters),
                90,
                8.0,
                new Color8Bit(Color.kFirstBlue)));

    this.arm =
        elevator.append(
            new MechanismLigament2d(
                "Arm",
                ElevatorArmConstants.kElevatorArmLength.in(Meters),
                0,
                4.0,
                new Color8Bit(Color.kFirstRed)));

    this.algaeIntakePose =
        new Pose3d(
            VisualizerConstants.algaeRoot3d,
            new Rotation3d(0, AlgaeIntakePivotConstants.kPivotStartingAngle.in(Radians), 0));
  }

  public void update() {
    arm.setAngle(armSetpoint.get().in(Degrees));
    elevator.setLength(elevatorSetpoint.get().in(Meters));
    SmartDashboard.putData("Mech2d", mechanism);

    double startHeightToSetpoint =
        elevatorSetpoint.get().in(Meters) - ElevatorConstants.kElevatorMinimumHeight.in(Meters);

    // second stage moves twice as much as the first stage
    double firstStageHeight = (startHeightToSetpoint * 1 / 2.0);
    double secondStageHeight = (startHeightToSetpoint * 1.0);

    this.elevatorFirstStagePose =
        new Pose3d(
            VisualizerConstants.elevatorRoot3d.plus(new Translation3d(0, 0, firstStageHeight)),
            new Rotation3d());

    this.elevatorSecondStagePose =
        new Pose3d(
            VisualizerConstants.elevatorRoot3d.plus(new Translation3d(0, 0, secondStageHeight)),
            new Rotation3d());

    this.shoulderPose =
        new Pose3d(
            VisualizerConstants.armRoot3d.plus(new Translation3d(0, 0, startHeightToSetpoint)),
            new Rotation3d(0, Radians.convertFrom(-arm.getAngle() - 90, Degrees), 0));

    this.elbowPose =
        shoulderPose.transformBy(
            new Transform3d(
                new Translation3d(0, ElevatorArmConstants.kElevatorArmLength.in(Meters), 0),
                new Rotation3d()));

    this.algaeIntakePose =
        new Pose3d(
            VisualizerConstants.algaeRoot3d,
            new Rotation3d(-algaePivotSetpoint.get().in(Radians), 0, 0));
  }

  @Logged(name = "ElevatorFirstStage")
  public Pose3d getElevatorFirstStagePose() {
    return elevatorFirstStagePose;
  }

  @Logged(name = "ElevatorSecondStage")
  public Pose3d getElevatorSecondStagePose() {
    return elevatorSecondStagePose;
  }

  @Logged(name = "ShoulderPose")
  public Pose3d getShoulderPose() {
    return shoulderPose;
  }

  @Logged(name = "ElbowPose")
  public Pose3d getElbowPose() {
    return elbowPose;
  }

  @Logged(name = "AlgaeIntakePose")
  public Pose3d getWristPose() {
    return algaeIntakePose;
  }

  @Logged(name = "AllComponentPoses")
  public Pose3d[] getAllPoses() {
    return new Pose3d[] {
      elevatorFirstStagePose, elevatorSecondStagePose, shoulderPose, algaeIntakePose
    };
  }

  @Logged(name = "ElevatorSetpoint")
  public double getElevatorSetpoint() {
    return elevatorSetpoint.get().in(Meters);
  }

  @Logged(name = "ArmSetpoint")
  public double getArmSetpoint() {
    return armSetpoint.get().in(Degrees);
  }

  @Logged(name = "AlgaeIntakeSetpoint")
  public double getAlgaeIntakeSetpoint() {
    return algaePivotSetpoint.get().in(Degrees);
  }

  @Override
  public void periodic() {
    update();
  }
}
