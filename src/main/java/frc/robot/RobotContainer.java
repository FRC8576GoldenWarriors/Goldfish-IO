package frc.robot;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Commands.SwerveDrive;
import frc.robot.Subsystems.Arm.Arm;
import frc.robot.Subsystems.Arm.Arm.positions;
import frc.robot.Subsystems.Arm.ArmConstants;
import frc.robot.Subsystems.Arm.ArmIOSparkMax;
import frc.robot.Subsystems.Climb.Climb;
import frc.robot.Subsystems.Climb.ClimbConstants;
import frc.robot.Subsystems.Climb.ClimbIOSparkMax;
import frc.robot.Subsystems.EndEffector.EndEffector;
import frc.robot.Subsystems.EndEffector.EndEffectorConstants;
import frc.robot.Subsystems.EndEffector.EndEffectorIOSparkMax;
import frc.robot.Subsystems.GroundIntake.GroundIntake;
import frc.robot.Subsystems.GroundIntake.GroundIntakeConstants;
import frc.robot.Subsystems.GroundIntake.GroundIntakeIOSparkMax;
import frc.robot.Subsystems.GroundIntake.GroundIntake.states;
import frc.robot.Subsystems.Shintake.Shintake;
import frc.robot.Subsystems.Shintake.ShintakeIOSparkMax;
import frc.robot.Subsystems.SwerveDrive.Drivetrain;

public class RobotContainer {

  //   public static final Drivetrain m_drivetrain = Drivetrain.getInstance();

  public static final CommandXboxController driverController =
      new CommandXboxController(0);
  public static final CommandXboxController operatorController =
      new CommandXboxController(1);

  public final JoystickButton resetHeading_Start =
      new JoystickButton(driverController.getHID(), XboxController.Button.kStart.value);

  public final SendableChooser<Command> autoChooser;

  public static Drivetrain m_drivetrain;
  public static Shintake m_shintake;
  public static GroundIntake m_GroundIntake;
  public static EndEffector m_EndEffector;
  public static Climb m_Climb;
  public static Arm m_arm;

  ;

  public RobotContainer() {

  
      // System.out.println("is real");
      m_drivetrain = Drivetrain.getInstance();
      m_shintake = new Shintake(new ShintakeIOSparkMax());
      m_GroundIntake = new GroundIntake(new GroundIntakeIOSparkMax());
      m_EndEffector = new EndEffector(new EndEffectorIOSparkMax());
      m_Climb = new Climb(new ClimbIOSparkMax());
      m_arm = new Arm(new ArmIOSparkMax());

    //     m_DriverCamera =
    //         new Camera(Constants.VisionConstants.CameraConstants.DRIVER_CAMERA_NAME, 320, 240,
    //   30, true);
    //     m_CageCamera =
    //         new Camera(Constants.VisionConstants.CameraConstants.CAGE_CAMERA_NAME, 320, 240, 30,
    //   true);
    //     m_CameraStream = new DriverStream(m_DriverCamera, m_CageCamera);

      
      m_drivetrain.setDefaultCommand(new SwerveDrive());
    
      registerNamedCommands();


    
    // Add all the choices of Autonomous modes to the Smart Dashboar
    autoChooser = AutoBuilder.buildAutoChooser();

    configureBindings();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureBindings() {

      resetHeading_Start.onTrue(new InstantCommand(m_drivetrain::zeroHeading, m_drivetrain));

      // Driver controller
      // driverController.b().onTrue(Commands.run(()->m_arm.setWantedPosition(ArmConstants.ControlConstants.A2Position),m_arm));
      //driverController.y().onTrue(Commands.run(()->m_arm.setWantedPosition(ArmConstants.ControlConstants.A2Position),m_arm));
      // driverController.rightBumper().onTrue(Commands.run(()->m_arm.setWantedPosition(-1)));
      //Stops working after 1 use??(Add requirements to instant commands)
      // driverController.a().onTrue(Commands.run(()->m_GroundIntake.setGroundIntake(0.23, GroundIntakeConstants.ControlConstants.algaeInSpeed),m_GroundIntake));
      // driverController.x().onTrue(Commands.run(()->m_GroundIntake.setGroundIntake(GroundIntakeConstants.ControlConstants.groundIntakeUpPosition,0),m_GroundIntake));
      // driverController.y().onTrue(Commands.parallel(Commands.parallel(Commands.run(()->m_arm.setWantedPosition(ArmConstants.ControlConstants.A1Position),m_arm),Commands.run(()->m_EndEffector.setWantedSpeed(EndEffectorConstants.ControlConstants.pincherAlgaeSpeed),m_EndEffector))));
      driverController.a().onTrue(Commands.run(()->m_GroundIntake.setWantedState(states.Intake),m_GroundIntake));
      driverController.x().onTrue(Commands.run(()->m_GroundIntake.setWantedState(states.Rest),m_GroundIntake));
      driverController.rightBumper().onTrue(Commands.run(()->m_GroundIntake.setWantedState(states.Rest),m_GroundIntake));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void registerNamedCommands() {

  }

  
}
