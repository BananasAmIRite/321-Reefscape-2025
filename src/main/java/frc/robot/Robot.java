/* (C) Robolancers 2025 */
package frc.robot;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.ProcessorAlign;
import frc.robot.commands.ReefAlign;
import frc.robot.commands.StationAlign;
import frc.robot.util.VirtualSubsystem;

@Logged
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  @Logged(name = "RobotContainer")
  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();
    Epilogue.bind(this);

    /*
     * RobotConstants.kAprilTagFieldLayout takes a significant amount of computing to load,
     * referencing `RobotConstants.class` here forces the field layout to load instead of stalling
     * autonomousInit()/teleopInit()
     */
    @SuppressWarnings("unused")
    final var robotConstants = RobotConstants.class;

    // TODO: load robot alliance globally on DS connection to avoid extra lookups
    ReefAlign.loadReefAlignmentPoses();
    StationAlign.loadStationAlignmentPoses();
    ProcessorAlign.loadProcessorAlignmentPoses();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    /*
     * TODO: note that VirtualSubsystem periodics must run after Subsystem periodics
     * since inputs need to be defined before SuperstructureVisualizer references them
     */
    VirtualSubsystem.periodicAll();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
