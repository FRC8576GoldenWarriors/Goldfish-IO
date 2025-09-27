package frc.lib.drivers;

import edu.wpi.first.wpilibj.Preferences;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class LoggedTunableDouble implements DoubleSupplier, Periodical {

  public String path = "Tuning/";
  private Runnable runnable;
  private double value;
  private boolean isTuningMode;
  private boolean useSavedValue;
  private LoggedNetworkNumber loggedNetworkNumber;

  // ! untested
  public LoggedTunableDouble(
      String name,
      double defaultValue,
      boolean isTuningMode,
      boolean useSavedValue,
      Runnable runnable) {
    this(name, defaultValue, isTuningMode, useSavedValue);
    this.runnable = runnable;
  }

  public LoggedTunableDouble(
      String name, double defaultValue, boolean isTuningMode, boolean useSavedValue) {
    this.value = defaultValue;
    this.path += name;
    this.isTuningMode = isTuningMode;
    this.useSavedValue = useSavedValue;
    this.loggedNetworkNumber = new LoggedNetworkNumber(path, defaultValue);

    if (Preferences.containsKey(name) && useSavedValue) {
      value = Preferences.getDouble(name, defaultValue);
    } else if (!Preferences.containsKey(name) && useSavedValue) {
      Preferences.initDouble(name, defaultValue);
    }

    PeriodicalUtil.registerPeriodic(this);
  }

  public LoggedTunableDouble onChange(Runnable runnable) {
    this.runnable = runnable;
    return this;
  }

  @Override
  public double getAsDouble() {
    return value;
  }

  public void set(double newValue) {
    value = newValue;
    runnable.run();

    if (useSavedValue) Preferences.setDouble(path, value);
  }

  @Override
  public void periodic() {
    if (isTuningMode) {
      double loggedValue = loggedNetworkNumber.get();
      if (loggedValue != value) {
        this.set(loggedValue);
      }
    }
  }
}
