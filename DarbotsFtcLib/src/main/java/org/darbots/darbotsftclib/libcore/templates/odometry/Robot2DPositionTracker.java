package org.darbots.darbotsftclib.libcore.templates.odometry;

import org.darbots.darbotsftclib.libcore.calculations.dimentionalcalculation.Robot2DPositionIndicator;
import org.darbots.darbotsftclib.libcore.calculations.dimentionalcalculation.XYPlaneCalculations;

public interface Robot2DPositionTracker {

    Robot2DPositionIndicator getInitialPos();
    public Robot2DPositionIndicator getCurrentPosition();

    double getRobotWidth();
    void setRobotWidth(double RobotWidth);
    double getRobotHeight();
    void setRobotHeight(double Height);

    Robot2DPositionIndicator getRobotAxisLeftTopExtremePoint();
    Robot2DPositionIndicator getRobotAxisRightTopExtremePoint();
    Robot2DPositionIndicator getRobotAxisLeftBottomExtremePoint();
    Robot2DPositionIndicator getRobotAxisRightBottomExtremePoint();
    Robot2DPositionIndicator fieldAxisFromRobotAxis(Robot2DPositionIndicator RobotAxisPoint);
    Robot2DPositionIndicator robotAxisFromFieldAxis(Robot2DPositionIndicator FieldAxisPoint);

    void resetRelativeOffset();
    Robot2DPositionIndicator getRelativeOffset();
}
