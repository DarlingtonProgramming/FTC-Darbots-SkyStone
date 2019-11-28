package org.darbots.darbotsftclib.libcore.templates.chassis_related;

import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPose2D;

public interface RobotMotionSystemTaskCallBack {
    void taskFinished(RobotMotionSystem motionSystem, RobotPose2D startPosition, RobotPose2D endPosition, RobotPose2D RelativePositionMoved);
}
