package frc.robot.Subsystems.Shintake;

public class ShintakeConstants {
  public static class HardwareConstants {

    public static final int rollerMotorLowID = 32;
    public static final boolean rollerMotorLowIsInverted = true;

    public static final int rollerMotorHighID = 33;
    public static final boolean rollerMotorHighIsInverted = false;

    public static final double pivotEncoderFullRange = 1.0;
    public static final double pivotEncoderZero = 0.0;

    public static final int lowerRollerDigitalInputDIO = 8;
  }

  public static class ControlConstants {
    public static final int shootBottomRPM = 5300;
    public static final int shootUpperRPM = 4400;
  }
}
