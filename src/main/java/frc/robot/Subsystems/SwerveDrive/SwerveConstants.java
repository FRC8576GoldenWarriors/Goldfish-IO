package frc.robot.Subsystems.SwerveDrive;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class SwerveConstants {

  public static class DriverConstants {
    public static final double xDeadband = 0.07;
    public static final double yDeadband = 0.07;
    public static final double turnDeadband = 0.08;
    public static final double xCoefficient = 1.75;
    public static final double yCoefficient = 1.75;
    public static final double turnCoefficient = 1.675;
  }

  // 24K constants
  // public static final int LEFT_FRONT_DRIVE_ID = 7; // 2; //6
  // public static final int RIGHT_FRONT_DRIVE_ID = 1; // 4;
  // public static final int LEFT_BACK_DRIVE_ID = 4; // 6; // 5
  // public static final int RIGHT_BACK_DRIVE_ID = 3; // 8; // 2

  // public static final int LEFT_FRONT_TURN_ID = 6; // 1; //5
  // public static final int RIGHT_FRONT_TURN_ID = 8; // 3;
  // public static final int LEFT_BACK_TURN_ID = 5;
  // public static final int RIGHT_BACK_TURN_ID = 2; // 7; //1

  // public static final int LEFT_FRONT_CANCODER_ID = 3; // 0;
  // public static final int RIGHT_FRONT_CANCODER_ID = 4; // 1;
  // public static final int LEFT_BACK_CANCODER_ID = 0; // 2; // 2
  // public static final int RIGHT_BACK_CANCODER_ID = 1; // 3;

  // public static final int PIGEON_ID = 0;

  // public static double LEFT_FRONT_OFFSET =
  //     0.239990; // -0.240723;//0.476074;//-0.482422;//-0.344971;//0.228027;
  // public static double RIGHT_FRONT_OFFSET =
  //     0.269287; // -0.484131;//-0.482178;//-0.397217;//-0.099609;
  // public static double LEFT_BACK_OFFSET =
  //     0.230469; // 0.229248;//0.243652;//-0.036865;//0.013184;//0.032959;//-0.000244;
  // public static double RIGHT_BACK_OFFSET =
  //     -0.023682; // -0.347412;//0.466309;//0.479736;//-0.324463;//-0.113525;

  // public static boolean LEFT_FRONT_DRIVE_INVERTED = false;
  // public static boolean RIGHT_FRONT_DRIVE_INVERTED = true;
  // public static boolean LEFT_BACK_DRIVE_INVERTED = true;
  // public static boolean RIGHT_BACK_DRIVE_INVERTED = false;

  // public static boolean LEFT_FRONT_TURN_INVERTED = true;
  // public static boolean RIGHT_FRONT_TURN_INVERTED = true;
  // public static boolean LEFT_BACK_TURN_INVERTED = true;
  // public static boolean RIGHT_BACK_TURN_INVERTED = true;

  // Practice & comp things

  public static final int LEFT_FRONT_DRIVE_ID = 2;
  public static final int RIGHT_FRONT_DRIVE_ID = 4;
  public static final int LEFT_BACK_DRIVE_ID = 8;
  public static final int RIGHT_BACK_DRIVE_ID = 6;

  public static final int LEFT_FRONT_TURN_ID = 1;
  public static final int RIGHT_FRONT_TURN_ID = 3;
  public static final int LEFT_BACK_TURN_ID = 7;
  public static final int RIGHT_BACK_TURN_ID = 5;

  public static final int LEFT_FRONT_CANCODER_ID = 1;
  public static final int RIGHT_FRONT_CANCODER_ID = 2;
  public static final int LEFT_BACK_CANCODER_ID = 4;
  public static final int RIGHT_BACK_CANCODER_ID = 3;

  public static final int PIGEON_ID = 0;

  public static double LEFT_FRONT_OFFSET = -0.175049; // -0.160156; // 0.330322;
  public static double RIGHT_FRONT_OFFSET = 0.184326; // -0.299805; // 0.191406;
  public static double LEFT_BACK_OFFSET = -0.036865; // -0.039551; // -0.013428;
  public static double RIGHT_BACK_OFFSET = 0.173584; // 0.175293; // 0.178955;

  public static boolean LEFT_FRONT_DRIVE_INVERTED = false; // true;
  public static boolean RIGHT_FRONT_DRIVE_INVERTED = true; // false; // true;
  public static boolean RIGHT_BACK_DRIVE_INVERTED = false;
  public static boolean LEFT_BACK_DRIVE_INVERTED = false;

  public static boolean LEFT_FRONT_TURN_INVERTED = true;
  public static boolean RIGHT_FRONT_TURN_INVERTED = true;
  public static boolean RIGHT_BACK_TURN_INVERTED = true;
  public static boolean LEFT_BACK_TURN_INVERTED = true;

  public static double PRACTICE_LEFT_FRONT_OFFSET = 0.377930;
  public static double PRACTICE_RIGHT_FRONT_OFFSET = 0.094727;
  public static double PRACTICE_LEFT_BACK_OFFSET = 0.161377;
  public static double PRACTICE_RIGHT_BACK_OFFSET = 0.274658;

  public static boolean PRACTICE_LEFT_FRONT_DRIVE_INVERTED = true;
  public static boolean PRACTICE_RIGHT_FRONT_DRIVE_INVERTED = false;
  public static boolean PRACTICE_RIGHT_BACK_DRIVE_INVERTED = false;
  public static boolean PRACTICE_LEFT_BACK_DRIVE_INVERTED = true;

  public static boolean PRACTICE_LEFT_FRONT_TURN_INVERTED = true;
  public static boolean PRACTICE_RIGHT_FRONT_TURN_INVERTED = true;
  public static boolean PRACTICE_RIGHT_BACK_TURN_INVERTED = true;
  public static boolean PRACTICE_LEFT_BACK_TURN_INVERTED = true;

  public static final double WHEEL_DIAMETER = Units.inchesToMeters(4.00);
  public static final double DRIVE_MOTOR_GEAR_RATIO = 6.75;
  public static final double TURN_MOTOR_GEAR_RATIO = 150.0 / 7;
  public static final double DRIVE_MOTOR_PCONVERSION =
      WHEEL_DIAMETER * Math.PI / DRIVE_MOTOR_GEAR_RATIO;
  public static final double TURN_MOTOR_PCONVERSION =
      2 * Math.PI / TURN_MOTOR_GEAR_RATIO; // 2 * Math.PI
  public static final double DRIVE_MOTOR_VCONVERSION = DRIVE_MOTOR_PCONVERSION / 60.0;
  public static final double TURN_MOTOR_VCONVERSION = TURN_MOTOR_PCONVERSION / 60.0;
  public static final double KP_TURNING = 0.575;

  public static final double DRIVETRAIN_MAX_SPEED = 5.3; // 4.0, 5.5;
  public static final double DRIVETRAIN_MAX_ANGULAR_SPEED = 5 * Math.PI; // 3.5, 4.25, 5

  // Teleop constraints
  public static final double TELE_DRIVE_MAX_SPEED = DRIVETRAIN_MAX_SPEED / 0.85;
  public static final double TELE_DRIVE_MAX_ANGULAR_SPEED = DRIVETRAIN_MAX_ANGULAR_SPEED / 1.75;
  public static final double TELE_DRIVE_MAX_ACCELERATION = 7.5; // 3
  public static final double TELE_DRIVE_MAX_ANGULAR_ACCELERATION = 15; //

  public static final double AUTO_KP_TRANSLATION = 2.6; // 1.15
  public static final double AUTO_KP_ROTATIONAL = 5.5; // 0.07; // 0.1

  public static final int ROTATION_CURRENT_LIMIT = 30;
  public static final int DRIVE_CURRENT_LIMIT = 45;

  public static final double TRACK_WIDTH = Units.inchesToMeters(25.0); // Y WIDTH
  public static final double WHEEL_BASE = Units.inchesToMeters(25.0); // X LENGTH
  public static final double DRIVE_BASE_RADIUS =
      Math.sqrt((Math.pow(TRACK_WIDTH, 2) + Math.pow(WHEEL_BASE, 2))) / 2.0;
  // kevin pfeffer was here
  public static final PPHolonomicDriveController pid_controls =
      new PPHolonomicDriveController(
          new PIDConstants(AUTO_KP_TRANSLATION, 0.2, 0.1),
          new PIDConstants(AUTO_KP_ROTATIONAL, 0, 0.001));

  // CREATE NEW CONSTANTS FOR LENGTH AND WIDTH
  // Swerve Kinematics
  public static final SwerveDriveKinematics DRIVE_KINEMATICS =
      new SwerveDriveKinematics(
          new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
          new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
          new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
          new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2));
}
