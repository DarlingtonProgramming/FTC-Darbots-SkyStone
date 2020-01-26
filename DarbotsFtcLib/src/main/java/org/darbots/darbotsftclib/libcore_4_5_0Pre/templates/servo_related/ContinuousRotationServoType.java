package org.darbots.darbotsftclib.libcore_4_5_0Pre.templates.servo_related;

import org.darbots.darbotsftclib.libcore_4_5_0Pre.templates.motor_related.MotorType;

public abstract class ContinuousRotationServoType implements MotorType {
    public abstract String getServoName();
    public String getMotorName(){
        return getServoName();
    }
    @Override
    public double getCountsPerRev(){
        return 360;
    }
    public abstract double getPulseLowerInMicroSeconds();
    public abstract double getPulseUpperInMicroSeconds();
}
