package frc.robot.Subsystems.Arm;

public class ArmConstants {
  public static final class HardwareConstants {

    public static final int armMotorID = 21;
    public static final int armEncoderDIO = 3;
  }

  public static final class ControlConstants {

    public static final boolean motorIsInverted = true;

    public static final double kS = 0.020922; // 0.015;//0.020922
    public static final double kG = 0.016366; // 0.24;//0.016366
    public static final double kV = 0.00013576; // 0.0515;//0.00013576
    public static final double kA = 2.928 * Math.pow(10, -5); // 0;//2.928E-05

    public static final double kP = 40; // 0.21344;//38.0;//0.21344
    public static final double kI = 0; // 1.0;
    public static final double kD = 0.0021014; // 0.0;//0.0021014

    public static final double storedPosition = 0.03;

    public static final double A1Position = 0.31;
    public static final double A2Position = 0.41;
    public static final double LolipopPosition = 0.19;

    public static final double coralStationPosition = 0.675; // 0.66;
    public static final double L1Position = 0.14;
    public static final double L2Position = 0.21;
    public static final double L3Position = 0.30;

    public static final double handoffPosition = 0.735;

    public static final double lowSoftStopPositon = 0.0;
    public static final double highSoftStopPosition = 0.75;

    public static final double armEncoderOffset = -0.441;
    public static final boolean armEncoderIsInverted = true;

    public static final double COMOffset = 0.013194;
  }
}
