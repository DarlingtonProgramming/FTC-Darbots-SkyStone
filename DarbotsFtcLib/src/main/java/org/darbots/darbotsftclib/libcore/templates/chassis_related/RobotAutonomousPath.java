package org.darbots.darbotsftclib.libcore.templates.chassis_related;

import org.darbots.darbotsftclib.libcore.calculations.dimentionalcalculation.RobotPose2D;

public interface RobotAutonomousPath {
    RobotPose2D getPathEndPoint();
    void getError(RobotPose2D currentPosition, RobotPose2D errorReceiver);

    boolean isPathFinished(RobotPose2D currentPosition);
}
