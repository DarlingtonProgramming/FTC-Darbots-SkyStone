package org.darbots.darbotsftclib.libcore.templates.odometry;

import org.darbots.darbotsftclib.libcore.calculations.dimentionalcalculation.RobotPose2D;

public interface Robot2DPositionTracker {

    RobotPose2D getInitialPos();
    RobotPose2D getCurrentPosition();

    RobotPose2D getCurrentVelocityVector();

    void stop();

    RobotPose2D fieldAxisFromRobotAxis(RobotPose2D RobotAxisPoint);
    RobotPose2D robotAxisFromFieldAxis(RobotPose2D FieldAxisPoint);

    void resetRelativeOffset();
    RobotPose2D getRelativeOffset();
}
