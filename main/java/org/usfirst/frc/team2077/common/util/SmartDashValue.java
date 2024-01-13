package org.usfirst.frc.team2077.common.util;

import java.util.Optional;

public interface SmartDashValue<ValueType> {
    ValueType get();
    Optional<ValueType> getNullable();
    void set(ValueType to);
}
