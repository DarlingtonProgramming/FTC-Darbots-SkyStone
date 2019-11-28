package org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation;

public class RobotPoint2D {
	public double X;
    public double Y;
    public RobotPoint2D(double X, double Y) {
    	this.X = X;
    	this.Y = Y;
    }
    public RobotPoint2D(RobotPoint2D oldPoint) {
    	this.X = oldPoint.X;
    	this.Y = oldPoint.Y;
    }
    public RobotPoint2D(RobotPose2D indicator) {
    	this.X = indicator.X;
    	this.Y = indicator.Y;
    }
}
