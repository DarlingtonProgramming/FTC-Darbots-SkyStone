package org.darbots.darbotsftclib.libcore.templates.odometry;

import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPose2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotVector2D;

public interface Robot2DPositionTracker {

    RobotPose2D getInitialPos();
    RobotPose2D getCurrentPosition();

    RobotVector2D getCurrentVelocityVector();

    void stop();

    RobotPose2D fieldAxisFromRobotAxis(RobotPose2D RobotAxisPoint);
    RobotPose2D robotAxisFromFieldAxis(RobotPose2D FieldAxisPoint);
}
