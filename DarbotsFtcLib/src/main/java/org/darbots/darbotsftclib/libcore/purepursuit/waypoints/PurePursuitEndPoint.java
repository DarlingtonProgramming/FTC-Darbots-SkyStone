package org.darbots.darbotsftclib.libcore.purepursuit.waypoints;

import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPoint2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.XYPlaneCalculations;

public class PurePursuitEndPoint extends PurePursuitWayPoint {
    private double m_EndToleranceDist = 3;
    public boolean headingInterpolationEnabled;
    private double m_Heading;
    private double m_AllowedHeadingError = 5;

    public double getEndErrorToleranceDistance(){
        return this.m_EndToleranceDist;
    }

    public void setEndErrorToleranceDistance(double distance){
        this.m_EndToleranceDist = Math.abs(distance);
    }

    public double getDesiredHeading(){
        return this.m_Heading;
    }

    public void setDesiredHeading(double desiredHeading){
        this.m_Heading = XYPlaneCalculations.normalizeDeg(desiredHeading);
    }

    public double getAllowedHeadingError(){
        return this.m_AllowedHeadingError;
    }

    public void setAllowedHeadingError(double allowedHeadingError){
        this.m_AllowedHeadingError = Math.abs(allowedHeadingError);
    }

    public PurePursuitEndPoint(double X, double Y, boolean headingControl, double desiredHeading) {
        super(X, Y);
        this.headingInterpolationEnabled = headingControl;
        this.setDesiredHeading(desiredHeading);
    }

    public PurePursuitEndPoint(double X, double Y, double FollowRadius, double endFollowNormalizedSpeed, boolean headingControl, double desiredHeading) {
        super(X, Y, FollowRadius, endFollowNormalizedSpeed);
        this.headingInterpolationEnabled = headingControl;
        this.setDesiredHeading(desiredHeading);
    }

    public PurePursuitEndPoint(RobotPoint2D point,boolean headingControl, double desiredHeading) {
        super(point);
        this.headingInterpolationEnabled = headingControl;
        this.setDesiredHeading(desiredHeading);
    }

    public PurePursuitEndPoint(PurePursuitWayPoint point, boolean headingControl, double desiredHeading) {
        super(point);
        this.headingInterpolationEnabled = headingControl;
        this.setDesiredHeading(desiredHeading);
    }
}
