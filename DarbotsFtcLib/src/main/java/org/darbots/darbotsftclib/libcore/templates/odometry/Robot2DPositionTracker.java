package org.darbots.darbotsftclib.libcore.templates.odometry;

import org.darbots.darbotsftclib.libcore.calculations.dimentionalcalculation.Robot2DPositionIndicator;
import org.darbots.darbotsftclib.libcore.calculations.dimentionalcalculation.XYPlaneCalculations;

public interface Robot2DPositionTracker {

    Robot2DPositionIndicator getInitialPos();
    Robot2DPositionIndicator getCurrentPosition();

    Robot2DPositionIndicator getCurrentVelocityVector();

    void stop();

    Robot2DPositionIndicator fieldAxisFromRobotAxis(Robot2DPositionIndicator RobotAxisPoint);
    Robot2DPositionIndicator robotAxisFromFieldAxis(Robot2DPositionIndicator FieldAxisPoint);

    void resetRelativeOffset();
    Robot2DPositionIndicator getRelativeOffset();
}
