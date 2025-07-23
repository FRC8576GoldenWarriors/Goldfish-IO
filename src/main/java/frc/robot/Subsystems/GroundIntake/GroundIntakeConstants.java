package frc.robot.Subsystems.GroundIntake;

public class GroundIntakeConstants {
  public static class HardwareConstants {

    public static final int rollerMotorID = 30;
    public static final boolean rollerMotorIsInverted = false;

    public static final int pivotMotorID = 31;
    public static final boolean pivotMotorIsInverted = true;

    public static final int pivotEncoderDIO = 4;

    public static final int digitalInputDIO = 5;
  }

  public static class ControlConstants {
    public static final double groundIntakeUpPosition = 0.015; // 0.01;
    public static final double groundIntakeDownPosition = 0.15;

    public static final double algaeHoldPosition = 0.21;
    public static final double algaeInPosition = 0.23;

    public static final double algaeInSpeed = -1.0;
    public static final double algaeOutSpeed = 1.0;

    public static final double algaeShootSpeed = -0.45;

    public static final double coralDropSpeed = -0.2;

    public static final double pivotEncoderFullRange = 1.0;
    public static final double pivotEncoderZero = -0.07; // -0.16;
    public static final boolean pivotEncoderIsInverted = true;

    public static final double kP = 34; // 16; // 6.5;
    public static final double kI = 0;
    public static final double kD = 0.8;

    public static final double kS = 0.0; // 0.003;
    public static final double kA = 0;
    public static final double kG = 0.31; // 0.14;
    public static final double kV = 0.028; // 0.038;

    public static final double rollerIdlekS = -0.1;

    public static final double coastZone = 0.5;
  }
}
