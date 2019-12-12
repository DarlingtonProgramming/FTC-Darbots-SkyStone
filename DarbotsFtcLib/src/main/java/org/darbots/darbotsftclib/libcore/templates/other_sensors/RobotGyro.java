package org.darbots.darbotsftclib.libcore.templates.other_sensors;

import org.darbots.darbotsftclib.libcore.templates.RobotNonBlockingDevice;

public interface RobotGyro {
    public enum HeadingRotationPositiveOrientation{
        CounterClockwise,
        Clockwise
    }
    float getHeading();
    HeadingRotationPositiveOrientation getHeadingRotationPositiveOrientation();

}
