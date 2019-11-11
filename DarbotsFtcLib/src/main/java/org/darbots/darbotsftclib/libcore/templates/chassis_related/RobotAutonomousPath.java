package org.darbots.darbotsftclib.libcore.templates.chassis_related;

import org.darbots.darbotsftclib.libcore.calculations.dimentionalcalculation.Robot2DPositionIndicator;

public interface RobotAutonomousPath {
    Robot2DPositionIndicator getPathEndPoint();
    void getError(Robot2DPositionIndicator currentPosition, Robot2DPositionIndicator errorReceiver);

    boolean isPathFinished(Robot2DPositionIndicator currentPosition);
}
