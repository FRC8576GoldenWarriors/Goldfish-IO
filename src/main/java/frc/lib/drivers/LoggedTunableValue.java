package frc.lib.drivers;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class LoggedTunableValue implements DoubleSupplier, Periodical {

    public String path = "Tuning/";
    private Runnable runnable;
    private double value;
    private LoggedNetworkNumber loggedNetworkNumber;

    public LoggedTunableValue(String name, double defaultValue, Runnable runnable) {
        this(name, defaultValue);
        this.runnable = runnable;
    }
    public LoggedTunableValue(String name, double defaultValue) {
        this.value = defaultValue;
        this.path += name;
        loggedNetworkNumber = new LoggedNetworkNumber(path, defaultValue);
        PeriodicalUtil.registerPeriodic(this);
    }

    @Override
    public double getAsDouble() {
        return value;
    }

    public void set(double newValue) {
        value = newValue;
        runnable.run();
    }

    @Override
    public void periodic() {
        double loggedValue = loggedNetworkNumber.get();
        if(loggedValue != value) {
            this.set(loggedValue);
        }
    }

}
