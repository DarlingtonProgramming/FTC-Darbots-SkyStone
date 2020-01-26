package org.darbots.darbotsftclib.libcore_4_5_0Pre.templates.other_sensors;

public interface RobotGyro {
    public enum HeadingRotationPositiveOrientation{
        CounterClockwise,
        Clockwise
    }
    float getHeading();
    HeadingRotationPositiveOrientation getHeadingRotationPositiveOrientation();

}
