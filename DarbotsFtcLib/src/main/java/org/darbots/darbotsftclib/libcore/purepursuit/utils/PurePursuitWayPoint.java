package org.darbots.darbotsftclib.libcore.purepursuit.utils;

import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPoint2D;

public class PurePursuitWayPoint extends RobotPoint2D {
    public PurePursuitWayPoint(double X, double Y){
        super(X,Y);
    }
    public PurePursuitWayPoint(PurePursuitWayPoint point){
        super(point);
    }
}
