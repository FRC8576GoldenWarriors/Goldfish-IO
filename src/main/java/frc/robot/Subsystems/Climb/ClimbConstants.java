package frc.robot.Subsystems.Climb;

public class ClimbConstants {
  public static final class HardwareConstants {
    public static final int motorID = 40;
    public static final boolean motorIsInverted = true;
    public static final int climberEncoderDIO = 6;
    public static final boolean climberEncoderIsInverted = true;
  }

  public static final class ControlConstants {
    public static final double climbUpSpeed = 1.0;
    public static final double climbDownSpeed = -1.0;

    public static final double climberUpPosition = 0.156;

    public static final double climberEncoderOffset = -0.36;
  }
}
