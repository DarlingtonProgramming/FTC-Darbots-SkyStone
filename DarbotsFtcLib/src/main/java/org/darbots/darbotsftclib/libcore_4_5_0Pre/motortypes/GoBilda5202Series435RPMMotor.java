package org.darbots.darbotsftclib.libcore_4_5_0Pre.motortypes;

import org.darbots.darbotsftclib.libcore_4_5_0Pre.templates.motor_related.MotorType;

public class GoBilda5202Series435RPMMotor implements MotorType {
    @Override
    public String getMotorName() {
        return "GoBlida 5202 Series 435RPM Motor";
    }

    @Override
    public double getCountsPerRev() {
        return 383.6;
    }

    @Override
    public double getRevPerSec() {
        return 7.25; //435 RPM
    }
}
