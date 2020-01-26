package org.darbots.darbotsftclib.libcore_4_5_0Pre.motortypes;

import org.darbots.darbotsftclib.libcore_4_5_0Pre.templates.motor_related.MotorType;

public class RevHDHex40Motor implements MotorType {
    @Override
    public String getMotorName() {
        return "Rev HD Hex 40 Motor";
    }

    @Override
    public double getCountsPerRev() {
        return 1120;
    }

    @Override
    public double getRevPerSec() {
        return 2.5;
    }
}
