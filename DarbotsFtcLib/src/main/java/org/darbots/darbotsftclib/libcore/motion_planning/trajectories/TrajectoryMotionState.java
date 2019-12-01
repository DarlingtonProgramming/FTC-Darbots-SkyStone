package org.darbots.darbotsftclib.libcore.motion_planning.trajectories;

public class TrajectoryMotionState {
    public double xDisplacement;
    public double yDisplacement;
    public double xVelocity;
    public double yVelocity;
    public TrajectoryMotionState(double xDisplacement, double yDisplacement, double xVelocity, double yVelocity){
        this.xDisplacement = xDisplacement;
        this.yDisplacement = yDisplacement;
        this.xVelocity = xVelocity;
        this.yVelocity = yVelocity;
    }
    public TrajectoryMotionState(TrajectoryMotionState oldMotionState){
        this.xDisplacement = oldMotionState.xDisplacement;
        this.yDisplacement = oldMotionState.yDisplacement;
        this.xVelocity = oldMotionState.xVelocity;
        this.yVelocity = oldMotionState.yVelocity;
    }
}
