package org.darbots.darbotsftclib.libcore_4_5_0Pre.motortypes;

import org.darbots.darbotsftclib.libcore_4_5_0Pre.templates.motor_related.MotorType;

public class GoBilda5202Series60RPMMotor implements MotorType {
    @Override
    public String getMotorName() {
        return "GoBlida 5202 Series 60RPM Motor";
    }

    @Override
    public double getCountsPerRev() {
        return 696.5;
    }

    @Override
    public double getRevPerSec() {
        return 1;
    }
}
