package org.darbots.darbotsftclib.libcore.templates.odometry;

import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPose2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotVector2D;

public interface CustomizableOdometry {
    void __trackLoopMoved(RobotVector2D velocity, RobotPose2D deltaRobotAxis);
    void __trackLoopMovedRaw(RobotVector2D velocity, RobotPose2D deltaRobotAxis);
    RobotVector2D getDistanceFactors();
    double getRotZDistanceFactor();
    double getYDistanceFactor();
    double getXDistanceFactor();
    double __getDeltaAng(double supposedDeltaAng);
}
