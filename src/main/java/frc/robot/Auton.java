package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.Arm.Arm;
import frc.robot.Subsystems.Arm.Arm.ArmPositions;
import frc.robot.Subsystems.Climb.Climb;
import frc.robot.Subsystems.EndEffector.EndEffector;
import frc.robot.Subsystems.GroundIntake.GroundIntake;
import frc.robot.Subsystems.GroundIntake.GroundIntake.GroundIntakeStates;
import frc.robot.Subsystems.Macros;
import frc.robot.Subsystems.Macros.states;
import frc.robot.Subsystems.Shintake.Shintake;
import frc.robot.Subsystems.Shintake.Shintake.ShintakeStates;

public class Auton {
  private Arm m_Arm;

  @SuppressWarnings("unused")
  private Climb m_Climb;

  private EndEffector m_EndEffector;
  private GroundIntake m_GroundIntake;
  private Shintake m_Shintake;
  public final SendableChooser<Command> autoSelector;
  public boolean autonInverted;
  private Macros macros;

  public Auton(
      Arm m_Arm,
      Climb m_Climb,
      EndEffector m_EndEffector,
      GroundIntake m_GroundIntake,
      Shintake m_Shintake,
      Macros macros,
      boolean inverted) {
    this.m_Arm = m_Arm;
    this.m_Climb = m_Climb;
    this.m_EndEffector = m_EndEffector;
    this.m_GroundIntake = m_GroundIntake;
    this.m_Shintake = m_Shintake;
    autoSelector = AutoBuilder.buildAutoChooser();

    autonInverted = inverted;
    // autoSelector.setDefaultOption("Do nothing", null);
    autoSelector.addOption("Drive test", driveCommand());

    SmartDashboard.putData("Auto Chooser", autoSelector);
    this.macros = macros;
  }

  public SequentialCommandGroup driveCommand() {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(
            WarriorAuto("Path Test", autonInverted),
            Commands.run(() -> macros.setWantedState(states.A1IntakeAuto))
                .until(() -> m_EndEffector.getAlgaeInput())),
        new ParallelCommandGroup(
            WarriorAuto("M1 Shoot", autonInverted),
            Commands.run(() -> macros.setWantedState(states.A1HandOffAuto))
                .until(
                    () ->
                        m_Arm.getPosition() == ArmPositions.Idle
                            && m_GroundIntake.getState() == GroundIntakeStates.Hold
                            && m_Shintake.getState() == ShintakeStates.Rest)));
  }

  public Command WarriorAuto(String autoName, boolean mirrored) {
    Command autoCommand = new PathPlannerAuto(autoName, mirrored);
    return Commands.waitSeconds(0.01).andThen(autoCommand);
  }

  public Command getAutonomousCommand() {
    return autoSelector.getSelected();
  }
}
