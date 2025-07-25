// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

// import frc.robot.Subsystems.Drivetrain;
// import frc.robot.Subsystems.Simulation.SimConstants;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.urcl.URCL;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  // private Drivetrain m_drivetrain = Drivetrain.getInstance();
  private RobotContainer m_robotContainer;

  // private List<Integer> usableTags;

  @Override
  public void robotInit() {
    Logger.recordMetadata("Goldfish", "Goldfish"); // Set a metadata value

    if (isReal()) {
      // 8576.winWorlds();
      Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
      Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
      new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
      Logger.registerURCL(URCL.startExternal());
    }
    // } else {
    //   setUseTiming(false); // Run as fast as possible
    //   String logPath =
    //       LogFileUtil
    //           .findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
    //   Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
    //   Logger.addDataReceiver(
    //   Te amo MG
    //       new WPILOGWriter(
    //           LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
    // }

    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may
    // be added.

    FollowPathCommand.warmupCommand().schedule();
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    // usableTags =
    //     RobotContainer.reefTagStatsLimelight.isBlueAlliance()
    //         ? Constants.VisionConstants.aprilTagConstants.IDs.BLUE_TAG_IDS
    //         : Constants.VisionConstants.aprilTagConstants.IDs.RED_TAG_IDS;
    SmartDashboard.putNumber("Time", DriverStation.getMatchTime());
    SmartDashboard.putNumber("Voltage", RobotController.getBatteryVoltage());

    Logger.recordOutput("Robot/Match Time", DriverStation.getMatchTime());
    Logger.recordOutput("Robot/Battery Voltage", RobotController.getBatteryVoltage());
    
    CommandScheduler.getInstance().run();
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
    // IMPLEMENT
    // m_drivetrain.resetAllEncoders();
    // m_drivetrain.setAllIdleMode(true);

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    // m_drivetrain.zeroHeading();
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
    // m_drivetrain.setHeading((m_drivetrain.getHeading()+180));

    // Implement
    // if (SimConstants.currentMode.equals(SimConstants.Mode.REAL)) {
    //   m_drivetrain.resetAllEncoders();
    //   m_drivetrain.setAllIdleMode(true);
    // }
  }

  @Override
  public void teleopPeriodic() {
    if(DriverStation.getMatchTime()<135&&DriverStation.getMatchTime()>125){
      new StartEndCommand(()->RobotContainer.driverController.setRumble(RumbleType.kBothRumble, 1),()->RobotContainer.driverController.setRumble(RumbleType.kBothRumble, 0)).withTimeout(5);
    }
    // else{
    //   RobotContainer.driverController.setRumble(RumbleType.kBothRumble, 0);
    // }
  }

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
