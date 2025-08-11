package frc.robot;

import org.littletonrobotics.junction.Logger;

import com.fasterxml.jackson.annotation.JsonTypeInfo.Id;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Commands.SwerveDrive;
import frc.robot.Commands.VisionAutoAlign;
import frc.robot.Commands.VisionReefAlign;
import frc.robot.Commands.VisionReefAlign.reefAlignState;
import frc.robot.Subsystems.Macros;
import frc.robot.Subsystems.Arm.Arm;
import frc.robot.Subsystems.Arm.ArmConstants;
import frc.robot.Subsystems.Arm.ArmIOSparkMax;
import frc.robot.Subsystems.Arm.Arm.ArmPositions;
import frc.robot.Subsystems.Climb.Climb;
import frc.robot.Subsystems.Climb.ClimbConstants;
import frc.robot.Subsystems.Climb.ClimbIOSparkMax;
import frc.robot.Subsystems.Climb.Climb.climbStates;
import frc.robot.Subsystems.EndEffector.EndEffector;
import frc.robot.Subsystems.EndEffector.EndEffectorConstants;
import frc.robot.Subsystems.EndEffector.EndEffectorIOSparkMax;
import frc.robot.Subsystems.GroundIntake.GroundIntake;
import frc.robot.Subsystems.GroundIntake.GroundIntakeConstants;
import frc.robot.Subsystems.GroundIntake.GroundIntakeIOSparkMax;
import frc.robot.Subsystems.GroundIntake.GroundIntake.GroundIntakeStates;
import frc.robot.Subsystems.LEDs.LEDConstants;
import frc.robot.Subsystems.LEDs.LEDs;
import frc.robot.Subsystems.Macros.states;
import frc.robot.Subsystems.Shintake.Shintake;
import frc.robot.Subsystems.Shintake.ShintakeIOSparkMax;
import frc.robot.Subsystems.Shintake.Shintake.ShintakeStates;
import frc.robot.Subsystems.SwerveDrive.Drivetrain;
import frc.robot.Subsystems.Vision.Limelight;
import frc.robot.Subsystems.Vision.LimelightConstants;
import frc.robot.Subsystems.Vision.LimelightIO;

public class RobotContainer {

  //   public static final Drivetrain m_drivetrain = Drivetrain.getInstance();

  public static final CommandXboxController driverController =
      new CommandXboxController(0);
  public static final GenericHID operatorButtons =
      new GenericHID(1);

      //Triggers
  public final JoystickButton resetHeading_Start =
      new JoystickButton(driverController.getHID(), XboxController.Button.kStart.value);
     //Rumble Trigger:
  //final Trigger rumble = new Trigger(()->DriverStation.isTeleop()&&(DriverStation.getMatchTime()==20||DriverStation.getMatchTime()==21));
  public final Trigger reefAlignTrigger = new Trigger(()->(m_Arm.getPosition()==ArmPositions.A1||m_Arm.getPosition()==ArmPositions.A2)&&m_Limelight.hasTargets(LimelightConstants.NameConstants.REEF_NETWORKTABLE_KEY)&&driverController.leftBumper().getAsBoolean());
  public final Trigger bargeAlignTrigger = new Trigger(()->m_Limelight.hasTargets(LimelightConstants.NameConstants.BARGE_NETWORKTABLE_KEY)&&driverController.leftTrigger(0.5).getAsBoolean());

  public final SendableChooser<Command> autoChooser;

  public static Drivetrain m_Drivetrain;
  public static Shintake m_Shintake;
  public static GroundIntake m_GroundIntake;
  public static EndEffector m_EndEffector;
  public static Climb m_Climb;
  public static Arm m_Arm;
  public static LEDs m_LEDs;
  public static Limelight m_Limelight;

  public static Macros macros;

  public RobotContainer() {

  
      // System.out.println("is real");
      m_Drivetrain = Drivetrain.getInstance();
      m_Shintake = new Shintake(new ShintakeIOSparkMax());
      m_GroundIntake = new GroundIntake(new GroundIntakeIOSparkMax());
      m_EndEffector = new EndEffector(new EndEffectorIOSparkMax());
      m_Climb = new Climb(new ClimbIOSparkMax());
      m_Arm = new Arm(new ArmIOSparkMax());
      m_LEDs = new LEDs(LEDConstants.HardwareConstants.LED_PORT, LEDConstants.HardwareConstants.LED_LENGTH);
      m_Limelight = new Limelight(
        new LimelightIO(LimelightConstants.NameConstants.BARGE_NETWORKTABLE_KEY,
        LimelightConstants.PositionalConstants.BARGE_LIMELIGHT_LOCATION), 
        new LimelightIO(LimelightConstants.NameConstants.REEF_NETWORKTABLE_KEY,
        LimelightConstants.PositionalConstants.REEF_LIMELIGHT_LOCATION));
      macros = new Macros(m_Arm, m_Climb, m_EndEffector, m_GroundIntake, m_Shintake);
      
    //     m_DriverCamera =
    //         new Camera(Constants.VisionConstants.CameraConstants.DRIVER_CAMERA_NAME, 320, 240,
    //   30, true);
    //     m_CageCamera =
    //         new Camera(Constants.VisionConstants.CameraConstants.CAGE_CAMERA_NAME, 320, 240, 30,
    //   true);
    //     m_CameraStream = new DriverStream(m_DriverCamera, m_CageCamera);

      
      m_Drivetrain.setDefaultCommand(new SwerveDrive());
    
      registerNamedCommands();


    
    // Add all the choices of Autonomous modes to the Smart Dashboar
    autoChooser = AutoBuilder.buildAutoChooser();

    configureBindings();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureBindings() {

      resetHeading_Start.onTrue(new InstantCommand(m_Drivetrain::zeroHeading, m_Drivetrain));

      // Driver controller
      // driverController.b().onTrue(Commands.run(()->m_arm.setWantedPosition(ArmConstants.ControlConstants.A2Position),m_arm));
      //driverController.y().onTrue(Commands.run(()->m_arm.setWantedPosition(ArmConstants.ControlConstants.A2Position),m_arm));
      // driverController.rightBumper().onTrue(Commands.run(()->m_arm.setWantedPosition(-1)));
      //Stops working after 1 use??(Add requirements to instant commands)
      // driverController.a().onTrue(Commands.run(()->m_GroundIntake.setGroundIntake(0.23, GroundIntakeConstants.ControlConstants.algaeInSpeed),m_GroundIntake));
      // driverController.x().onTrue(Commands.run(()->m_GroundIntake.setGroundIntake(GroundIntakeConstants.ControlConstants.groundIntakeUpPosition,0),m_GroundIntake));
      // driverController.y().onTrue(Commands.parallel(Commands.parallel(Commands.run(()->m_arm.setWantedPosition(ArmConstants.ControlConstants.A1Position),m_arm),Commands.run(()->m_EndEffector.setWantedSpeed(EndEffectorConstants.ControlConstants.pincherAlgaeSpeed),m_EndEffector))));
      // driverController.a().onTrue(new InstantCommand(()->macros.setWantedState(states.A1),macros));
      // driverController.y().onTrue(new InstantCommand(()->macros.setWantedState(states.A2),macros));
      // driverController.b(). onTrue(new InstantCommand(()->macros.setWantedState(states.L1),macros));
      // driverController.rightBumper().onTrue(new InstantCommand(()->macros.setWantedState(states.Processor),macros));
      // driverController.leftBumper().onTrue(new InstantCommand(()->macros.setWantedState(states.Score),macros));
      driverController.povUp().onTrue(new InstantCommand(()->m_Climb.setClimbAngle(climbStates.VoltageControl),m_Climb));
      driverController.povDown().onTrue(new InstantCommand(()->m_Climb.setClimbAngle(climbStates.VoltageControl),m_Climb));
      driverController.y().onTrue(new InstantCommand(()->m_Climb.setClimbAngle(climbStates.ClimbUp),m_Climb));
      driverController.b().onTrue(new InstantCommand(()->m_Climb.setClimbAngle(climbStates.ClimbDown), m_Climb));

      driverController.rightTrigger(0.5).onTrue(new InstantCommand(()->macros.setWantedState(states.Score),macros));

      // driverController.leftTrigger(0.5).and(()->m_GroundIntake.getAlgaeDetected()||m_Arm.getPosition()==ArmPositions.Station).whileTrue(new VisionAutoAlign(m_Drivetrain, m_Limelight));
      bargeAlignTrigger.whileTrue(new VisionAutoAlign(m_Drivetrain, m_Limelight));
      reefAlignTrigger.whileTrue(new VisionReefAlign(m_Drivetrain, m_Limelight, reefAlignState.Middle));

      driverController.rightBumper().whileTrue(new VisionReefAlign(m_Drivetrain, m_Limelight, reefAlignState.RightSide));
      driverController.leftBumper().whileTrue  (new VisionReefAlign(m_Drivetrain, m_Limelight, reefAlignState.LeftSide));
      //Left Trigger for limelight align
      driverController.leftTrigger().whileTrue(new VisionAutoAlign(m_Drivetrain, m_Limelight));
      //Operator Button Board
      new Trigger(()->operatorButtons.getRawAxis(2)>=0.5).onTrue(new InstantCommand(()->macros.setWantedState(states.Processor),macros));
      new Trigger(()->operatorButtons.getRawButton(1)).onTrue(new InstantCommand(()->macros.setWantedState(states.GroundIntake),macros));
      new Trigger(()->operatorButtons.getRawAxis(3)>=0.5).onTrue(new InstantCommand(()->macros.setWantedState(states.A1),macros));
      new Trigger(()->operatorButtons.getRawButton(2)).onTrue(new InstantCommand(()->macros.setWantedState(states.A2),macros));

      new Trigger(()->operatorButtons.getRawButton(5)).onTrue(new InstantCommand(()->macros.setWantedState(states.L1),macros));
      new Trigger(()->operatorButtons.getRawButton(3)).onTrue(new InstantCommand(()->macros.setWantedState(states.L2),macros));
      new Trigger(()->operatorButtons.getRawButton(4)).onTrue(new InstantCommand(()->macros.setWantedState(states.L3),macros));
      new Trigger(()->operatorButtons.getRawButton(6)).onTrue(new InstantCommand(()->macros.setWantedState(states.Lolipop),macros));

      new Trigger(()->operatorButtons.getRawButton(7)).onTrue(new InstantCommand(()->macros.setWantedState(states.Wave),macros));
      new Trigger(()->operatorButtons.getRawButton(8)).onTrue(new InstantCommand(()->macros.setWantedState(states.Wave),macros));

      new Trigger(()->operatorButtons.getRawButton(10)).onTrue(new InstantCommand(()->macros.setWantedState(states.Rest),macros));
      
      //driverController.leftBumper().onFalse(new InstantCommand(()->m_Shintake.setWantedState(ShintakeStates.Rest),m_Shintake));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void registerNamedCommands() {
    NamedCommands.registerCommand("A1 Intake", new InstantCommand(()->macros.setWantedState(states.A1IntakeAuto),macros).until(()->m_EndEffector.getAlgaeInput()));
    NamedCommands.registerCommand("A1 Handoff", new StartEndCommand(()->macros.setWantedState(states.A1HandOffAuto),()->m_Arm.setWantedPosition(ArmPositions.Holding),macros).until(()->m_Arm.getPosition()==ArmPositions.Holding));
    NamedCommands.registerCommand("A2 Intake", new InstantCommand(()->macros.setWantedState(states.A2IntakeAuto),macros).until(()->m_EndEffector.getAlgaeInput()));
    NamedCommands.registerCommand("A2 Handoff", new StartEndCommand(()->macros.setWantedState(states.A2HandoffAuto),()->macros.setWantedState(states.GroundIntake),macros).until(()->m_GroundIntake.getState()==GroundIntakeStates.Hold));
    NamedCommands.registerCommand("Score", new InstantCommand(()->macros.setWantedState(states.Score),macros).until(()->m_GroundIntake.getState()==GroundIntakeStates.Rest));
    NamedCommands.registerCommand("L1", new InstantCommand(()->macros.setWantedState(states.L1),macros));
    NamedCommands.registerCommand("L2", new InstantCommand(()->macros.setWantedState(states.L2),macros));
    NamedCommands.registerCommand("L3", new InstantCommand(()->macros.setWantedState(states.L3),macros));
    NamedCommands.registerCommand("Slack",new StartEndCommand(()->m_Climb.setClimbAngle(climbStates.VoltSlack), ()->m_Climb.setClimbAngle(climbStates.Idle), m_Climb).withTimeout(1.7));
    NamedCommands.registerCommand("Align to Barge", new VisionAutoAlign(m_Drivetrain, m_Limelight).until(()->Math.abs(m_Limelight.getDistanceToTag(LimelightConstants.NameConstants.BARGE_NETWORKTABLE_KEY, true)-LimelightConstants.PhysicalConstants.DESIRED_APRIL_TAG_DISTANCE_BARGE)<0.7||!m_Limelight.hasTargets(LimelightConstants.NameConstants.BARGE_NETWORKTABLE_KEY)));
    NamedCommands.registerCommand("Align to Reef", new VisionReefAlign(m_Drivetrain,m_Limelight,reefAlignState.Middle).until(()->Math.abs(m_Limelight.getDistanceToTag(LimelightConstants.NameConstants.REEF_NETWORKTABLE_KEY, true)-LimelightConstants.PhysicalConstants.DESIRED_APRIL_TAG_DISTANCE_REEF)<0.3||!m_Limelight.hasTargets(LimelightConstants.NameConstants.REEF_NETWORKTABLE_KEY)));
  }

  
}
