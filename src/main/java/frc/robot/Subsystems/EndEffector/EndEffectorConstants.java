package frc.robot.Subsystems.EndEffector;

public class EndEffectorConstants {
  public static final class HardwareConstants {
    public static final int pincherID = 20;
    public static final int coralLeftDigiSensorID = 1;
    public static final int algaeDigiSensorID = 2;

    public static final boolean motorIsInverted = true;
  }

  public static final class ControlConstants {
    public static final double pincherAlgaeSpeed = -1.0; // -0.65

    public static final double pincherCoralInSpeed = 1.00;
    public static final double pincherCoralOutSpeed = -1.00;

    public static final double pincherCoralL3OutSpeed = -0.6;
  }
}
