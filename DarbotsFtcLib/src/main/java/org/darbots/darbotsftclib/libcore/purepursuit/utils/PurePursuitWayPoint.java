package org.darbots.darbotsftclib.libcore.purepursuit.utils;

import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPoint2D;

public class PurePursuitWayPoint extends RobotPoint2D {
    private double m_FollowDistance = 20;
    public double getFollowDistance(){
        return this.m_FollowDistance;
    }
    public void setFollowDistance(double followDistance){
        this.m_FollowDistance = Math.abs(followDistance);
    }
    public PurePursuitWayPoint(double X, double Y){
        super(X,Y);
    }
    public PurePursuitWayPoint(RobotPoint2D point){
        super(point);
    }
    public PurePursuitWayPoint(PurePursuitWayPoint point){
        super(point);
        this.m_FollowDistance = point.m_FollowDistance;
    }
}
