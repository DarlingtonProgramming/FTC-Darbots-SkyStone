package org.darbots.darbotsftclib.libcore.motion_planning.trajectories;

public class TrajectoryMotionSegment {
    public double xDisplacement;
    public double yDisplacement;
    public double xVelocity;
    public double yVelocity;
    public double xAcceleration;
    public double yAcceleration;
    public double duration;
    public TrajectoryMotionSegment(double xDisplacement, double yDisplacement, double xVelocity, double yVelocity, double xAcceleration, double yAcceleration, double duration){
        this.xDisplacement = xDisplacement;
        this.yDisplacement = yDisplacement;
        this.xVelocity = xVelocity;
        this.yVelocity = yVelocity;
        this.xAcceleration = xAcceleration;
        this.yAcceleration = yAcceleration;
        this.duration = duration;
    }
    public TrajectoryMotionSegment(TrajectoryMotionSegment oldMotionState){
        this.xDisplacement = oldMotionState.xDisplacement;
        this.yDisplacement = oldMotionState.yDisplacement;
        this.xVelocity = oldMotionState.xVelocity;
        this.yVelocity = oldMotionState.yVelocity;
        this.xAcceleration = oldMotionState.xAcceleration;
        this.yAcceleration = oldMotionState.yAcceleration;
        this.duration = oldMotionState.duration;
    }
    public TrajectoryMotionState getStatusAt(double time){
        return new TrajectoryMotionState(
                this.xDisplacement + this.xVelocity * time + this.xAcceleration * time * time / 2.0,
                this.yDisplacement + this.yVelocity * time + this.yAcceleration * time * time / 2.0,
                this.xVelocity + this.xAcceleration * time,
                this.yVelocity + this.yAcceleration * time
        );
    }
}
