package org.darbots.darbotsftclib.libcore.motortypes;

import org.darbots.darbotsftclib.libcore.templates.motor_related.MotorType;

public class GoBilda5202Series1150RPMMotor implements MotorType {
    @Override
    public String getMotorName() {
        return "GoBlida 5202 Series 1150RPM Motor";
    }

    @Override
    public double getCountsPerRev() {
        return 145.6;
    }

    @Override
    public double getRevPerSec() {
        return 19.167; //1150 rpm
    }
}
