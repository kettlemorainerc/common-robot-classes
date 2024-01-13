package org.usfirst.frc.team2077.common.util;

import edu.wpi.first.networktables.*;
import edu.wpi.first.networktables.NetworkTableEvent.Kind;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.*;

public class SmartDashNumber implements SmartDashValue<Double> {
    private static NetworkTable table = NetworkTableInstance.getDefault().getTable("SmartDashboard");

    private final NetworkTableEntry entry;
    private List<Runnable> onChange;
    private Double value;

    public SmartDashNumber(String key, Double defaultValue, boolean persistent) {
        if(SmartDashboard.getNumber(key, Double.MIN_VALUE) == Double.MIN_VALUE) {
            SmartDashboard.putNumber(key, defaultValue);
        }

        onChange = new LinkedList<>();

        entry = table.getEntry(key);
        value = defaultValue;

        if(persistent) entry.setPersistent();
        else entry.clearPersistent();

        var events = EnumSet.of(
                Kind.kImmediate,
                Kind.kValueAll
        );

        table.addListener(key, events, (networkTable, tableKey, event) -> {
            this.value = event.valueData.value.getDouble();
            onChange.forEach(Runnable::run);
        });
    }

    public void onChange(Runnable runnable) {
        this.onChange.add(runnable);
    }

    @Override public Double get() {
        return value;
    }

    @Override public Optional<Double> getNullable() {
        return Optional.ofNullable(value);
    }

    @Override public void set(Double to) {
        if(!Objects.equals(to, value)) entry.setNumber(to);
    }
}
