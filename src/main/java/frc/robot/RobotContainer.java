package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.SwerveDrive;
import frc.robot.Commands.VisionAutoAlign;
import frc.robot.Commands.VisionReefAlign;
import frc.robot.Commands.PoseBasedAligns.BargeAlign;
import frc.robot.Commands.VisionReefAlign.ReefAlignState;
import frc.robot.Subsystems.Macros;
import frc.robot.Subsystems.Arm.Arm;
import frc.robot.Subsystems.Arm.ArmIOSparkMax;
import frc.robot.Subsystems.Arm.Arm.ArmPositions;
import frc.robot.Subsystems.Climb.Climb;
import frc.robot.Subsystems.Climb.ClimbIOSparkMax;
import frc.robot.Subsystems.Climb.Climb.climbStates;
import frc.robot.Subsystems.EndEffector.EndEffector;
import frc.robot.Subsystems.EndEffector.EndEffectorIOSparkMax;
import frc.robot.Subsystems.GroundIntake.GroundIntake;
import frc.robot.Subsystems.GroundIntake.GroundIntakeIOSparkMax;
import frc.robot.Subsystems.GroundIntake.GroundIntake.GroundIntakeStates;
import frc.robot.Subsystems.LEDs.LEDConstants;
import frc.robot.Subsystems.LEDs.LEDs;
import frc.robot.Subsystems.Macros.states;
import frc.robot.Subsystems.Shintake.Shintake;
import frc.robot.Subsystems.Shintake.ShintakeIOSparkMax;
import frc.robot.Subsystems.SwerveDrive.Drivetrain;
import frc.robot.Subsystems.SwerveDrive.Gyro.GyroPidgeonIO;
import frc.robot.Subsystems.SwerveDrive.Module.ModuleIOSparkMax;
import frc.robot.Subsystems.Vision.TagMap;
import frc.robot.Subsystems.Vision.TagMap.Tags;
import frc.robot.Subsystems.Vision.Limelight.Limelight;
import frc.robot.Subsystems.Vision.Limelight.LimelightConstants;
import frc.robot.Subsystems.Vision.Limelight.LimelightIO;
import frc.robot.Subsystems.Vision.PhotonVision.PhotonVision;
import frc.robot.Subsystems.Vision.PhotonVision.PhotonVisionConstants;
import frc.robot.Subsystems.Vision.PhotonVision.PhotonVisionIO;

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
  public final Trigger bargeAlignTrigger = new Trigger(()->driverController.leftTrigger(0.5).getAsBoolean()); //m_Limelight.hasTargets(LimelightConstants.NameConstants.BARGE_NETWORKTABLE_KEY)&&
  public final Trigger redAlgaeTrigger = new Trigger(()->m_Limelight.hasTargets(LimelightConstants.NameConstants.BARGE_NETWORKTABLE_KEY)&&driverController.rightBumper().getAsBoolean());
  
  //public final LoggedDashboardChooser<Command> autoChooser;

  public static Drivetrain m_Drivetrain;
  public static Shintake m_Shintake;
  public static GroundIntake m_GroundIntake;
  public static EndEffector m_EndEffector;
  public static Climb m_Climb;
  public static Arm m_Arm;
  public static LEDs m_LEDs;
  public static Limelight m_Limelight;
  public static TagMap m_TagMap;
  public static PhotonVision m_PhotonVision;
  public static Auton m_Auton;

  public static Macros macros;

  public RobotContainer() {

  
      // System.out.println("is real");
      m_Drivetrain = new Drivetrain(new GyroPidgeonIO(), new ModuleIOSparkMax(1), new ModuleIOSparkMax(2), new ModuleIOSparkMax(3), new ModuleIOSparkMax(4));
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
      m_TagMap = new TagMap(AprilTagFields.k2025ReefscapeAndyMark, Tags.REEF);
      m_PhotonVision = new PhotonVision(
        new PhotonVisionIO(
          PhotonVisionConstants.NameConstants.LEFT_CAMERA, 
          PhotonVisionConstants.PositionalConstants.LEFT_CAMERA_LOCATION), 
        new PhotonVisionIO(
          PhotonVisionConstants.NameConstants.RIGHT_CAMERA, 
          PhotonVisionConstants.PositionalConstants.RIGHT_CAMERA_LOCATION));
      macros = new Macros(m_Arm, m_Climb, m_EndEffector, m_GroundIntake, m_Shintake);
      m_Auton = new Auton(m_Arm, m_Climb, m_EndEffector, m_GroundIntake, m_Shintake,macros, m_Drivetrain.isRedAlliance());
    //     m_DriverCamera =
    //         new Camera(Constants.VisionConstants.CameraConstants.DRIVER_CAMERA_NAME, 320, 240,
    //   30, true);
    //     m_CageCamera =
    //         new Camera(Constants.VisionConstants.CameraConstants.CAGE_CAMERA_NAME, 320, 240, 30,
    //   true);
    //     m_CameraStream = new DriverStream(m_DriverCamera, m_CageCamera);

      
      m_Drivetrain.setDefaultCommand(new SwerveDrive());
      registerNamedCommands();
      // autoChooser = AutoBuilder.buildAutoChooser();

    configureBindings();
    // SmartDashboard.putData("Auto Chooser", autoChooser);
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
      //driverController.povLeft().whileTrue( m_TagMap.AlignToClosestTag(m_Drivetrain, m_Limelight));
      driverController.povLeft().onTrue(new InstantCommand(() -> m_TagMap.recordIdealDistance()));
      // driverController.povRight().onTrue(new InstantCommand(() -> m_Drivetrain.setPose(m_Limelight.getPose2d(LimelightConstants.NameConstants.BARGE_NETWORKTABLE_KEY))));

      driverController.rightTrigger(0.5).onTrue(new InstantCommand(()->macros.setWantedState(states.Score),macros));

      // driverController.leftTrigger(0.5).and(()->m_GroundIntake.getAlgaeDetected()||m_Arm.getPosition()==ArmPositions.Station).whileTrue(new VisionAutoAlign(m_Drivetrain, m_Limelight));
      bargeAlignTrigger.whileTrue(new BargeAlign(m_Drivetrain, m_Limelight, m_TagMap));
      reefAlignTrigger.whileTrue(new VisionReefAlign(m_Drivetrain, m_Limelight, ReefAlignState.Middle));

      redAlgaeTrigger.whileTrue(new VisionAutoAlign(m_Drivetrain, m_Limelight, true));
      driverController.leftBumper().whileTrue  (new VisionReefAlign(m_Drivetrain, m_Limelight, ReefAlignState.LeftSide));
      //Left Trigger for limelight align
      //driverController.leftTrigger().whileTrue(new VisionAutoAlign(m_Drivetrain, m_Limelight, false));
      //Operator Button Board
      new Trigger(()->operatorButtons.getRawAxis(2)>=0.5).onTrue(new InstantCommand(()->macros.setWantedState(states.Processor),macros));
      new Trigger(()->operatorButtons.getRawButton(1)).onTrue(new InstantCommand(()->macros.setWantedState(states.GroundIntake),macros));
      new Trigger(()->operatorButtons.getRawAxis(3)>=0.5).onTrue(new InstantCommand(()->macros.setWantedState(states.A1),macros));
      new Trigger(()->operatorButtons.getRawButton(2)).onTrue(new InstantCommand(()->macros.setWantedState(states.A2),macros));

      new Trigger(()->operatorButtons.getRawButton(5)).onTrue(new InstantCommand(()->macros.setWantedState(states.L1),macros));
      new Trigger(()->operatorButtons.getRawButton(3)).onTrue(new InstantCommand(()->macros.setWantedState(states.L2),macros));
      new Trigger(()->operatorButtons.getRawButton(4)).onTrue(new InstantCommand(()->macros.setWantedState(states.L3),macros));
      new Trigger(()->operatorButtons.getRawButton(6)).onTrue(new InstantCommand(()->macros.setWantedState(states.Lolipop),macros));

      driverController.a().onTrue(new InstantCommand(()->m_Climb.setClimbAngle(climbStates.Slack), m_Climb));
      new Trigger(()->operatorButtons.getRawButton(8)).onTrue(new InstantCommand(()->m_Climb.setClimbAngle(climbStates.ClimbDown), m_Climb));

      new Trigger(()->operatorButtons.getRawButton(10)).onTrue(new InstantCommand(()->macros.setWantedState(states.Rest),macros));

      
      
      //driverController.leftBumper().onFalse(new InstantCommand(()->m_Shintake.setWantedState(ShintakeStates.Rest),m_Shintake));
  }

  public Command getAutonomousCommand() {
    return  m_Auton.getAutonomousCommand();//new SequentialCommandGroup(m_Auton.WarriorAuto("Test Path", m_Drivetrain.isRedAlliance()));//m_Auton.getAutonomousCommand();//new SequentialCommandGroup(m_Auton.WarriorAuto("Test Path", m_Drivetrain.isRedAlliance()));
  }

  public void registerNamedCommands() {
    // NamedCommands.registerCommand("Reset Pose To Barge", new InstantCommand(() -> m_Drivetrain.setPose(m_Limelight.getPose2d(LimelightConstants.NameConstants.BARGE_NETWORKTABLE_KEY))));
    // NamedCommands.registerCommand("Reset Pose To Reef", new InstantCommand(() -> m_Drivetrain.setPose(m_Limelight.getPose2d(LimelightConstants.NameConstants.REEF_NETWORKTABLE_KEY))));
    NamedCommands.registerCommand("A1 Intake", new InstantCommand(()->macros.setWantedState(states.A1IntakeAuto),macros).until(()->m_EndEffector.getAlgaeInput()));
    NamedCommands.registerCommand("A1 Handoff", new StartEndCommand(()->macros.setWantedState(states.A1HandOffAuto),()->m_Arm.setWantedPosition(ArmPositions.Holding),macros).until(()->m_Arm.getPosition()==ArmPositions.Idle&&m_GroundIntake.getState()==GroundIntakeStates.Hold&&m_Shintake.getState()==ShintakeStates.Rest));//m_Arm.getPosition()==ArmPositions.Holding));
    NamedCommands.registerCommand("A2 Intake", new InstantCommand(()->macros.setWantedState(states.A2IntakeAuto),macros).until(()->m_EndEffector.getAlgaeInput()));
    NamedCommands.registerCommand("A2 Handoff", new StartEndCommand(()->macros.setWantedState(states.A2HandoffAuto),()->macros.setWantedState(states.GroundIntake),macros).until(()->m_Arm.getPosition()==ArmPositions.Idle&&m_GroundIntake.getState()==GroundIntakeStates.Hold));
    NamedCommands.registerCommand("Score", new InstantCommand(()->macros.setWantedState(states.Score),macros).until(()->m_GroundIntake.getState()==GroundIntakeStates.Rest&&!m_Shintake.shootersRevved()));
    NamedCommands.registerCommand("L1", new InstantCommand(()->macros.setWantedState(states.L1),macros));
    NamedCommands.registerCommand("L2", new InstantCommand(()->macros.setWantedState(states.L2),macros));
    NamedCommands.registerCommand("L3", new InstantCommand(()->macros.setWantedState(states.L3),macros));
    NamedCommands.registerCommand("Slack",new StartEndCommand(()->m_Climb.setClimbAngle(climbStates.VoltSlack), ()->m_Climb.setClimbAngle(climbStates.Idle), m_Climb).withTimeout(1.7));
    NamedCommands.registerCommand("Align to Barge", new VisionAutoAlign(m_Drivetrain, m_Limelight,false).until(()->Math.abs(m_Drivetrain.getDistanceToTagMeters(m_Limelight.getTagID(LimelightConstants.NameConstants.BARGE_NETWORKTABLE_KEY))-LimelightConstants.PhysicalConstants.DESIRED_APRIL_TAG_DISTANCE_BARGE)<0.7||!m_Limelight.hasTargets(LimelightConstants.NameConstants.BARGE_NETWORKTABLE_KEY)));
    NamedCommands.registerCommand("Align to Reef", new VisionReefAlign(m_Drivetrain,m_Limelight,ReefAlignState.Middle).until(()->Math.abs(m_Drivetrain.getDistanceToTagMeters(m_Limelight.getTagID(LimelightConstants.NameConstants.REEF_NETWORKTABLE_KEY))-LimelightConstants.PhysicalConstants.DESIRED_APRIL_TAG_DISTANCE_REEF)<0.45||!m_Limelight.hasTargets(LimelightConstants.NameConstants.REEF_NETWORKTABLE_KEY)||m_EndEffector.getAlgaeInput()));
    NamedCommands.registerCommand("Reset Speeds", new InstantCommand(()->m_Drivetrain.driveRobotRelative(new ChassisSpeeds())));
  }

  
}
