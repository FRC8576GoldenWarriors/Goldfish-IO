package frc.lib.drivers;

import edu.wpi.first.math.controller.PIDController;

public class TunablePIDController extends PIDController {

  private LoggedTunableDouble kP;
  private LoggedTunableDouble kI;
  private LoggedTunableDouble kD;
  private LoggedTunableDouble kSetpoint;

  // ! untested
  private TunablePIDController(
      String name, LoggedTunableDouble kP, LoggedTunableDouble kI, LoggedTunableDouble kD) {
    super(kP.getAsDouble(), kI.getAsDouble(), kD.getAsDouble());
    this.kP = kP.onChange(() -> this.setP(this.kP.getAsDouble()));
    this.kI = kI.onChange(() -> this.setI(this.kI.getAsDouble()));
    this.kD = kD.onChange(() -> this.setD(this.kD.getAsDouble()));
  }

  public TunablePIDController(
      String name, double kP, double kI, double kD, boolean useTuningMode, boolean useSavedValue) {
    this(
        name,
        new LoggedTunableDouble(name + " P", kP, useTuningMode, useSavedValue),
        new LoggedTunableDouble(name + " I", kI, useTuningMode, useSavedValue),
        new LoggedTunableDouble(name + " D", kD, useTuningMode, useSavedValue));
  }

  public TunablePIDController(
      String name,
      double kP,
      double kI,
      double kD,
      double kSetpoint,
      boolean useTuningMode,
      boolean useSavedValue) {
    this(name, kP, kI, kD, useTuningMode, useSavedValue);
    this.setSetpoint(kSetpoint);
    this.kSetpoint =
        new LoggedTunableDouble(name + " Setpoint", kSetpoint, useTuningMode, useSavedValue);
    this.kSetpoint.onChange(() -> this.setSetpoint(this.kSetpoint.getAsDouble()));
  }
}
