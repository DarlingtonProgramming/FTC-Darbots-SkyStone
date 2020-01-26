package org.darbots.darbotsftclib.libcore_4_5_0Pre.motion_planning.profiles;

public class MotionState {
    public double distance;
    public double velocity;
    public double acceleration;
    public double jerk;
    public MotionState(double distance, double velocity, double acceleration, double jerk){
        this.distance = distance;
        this.velocity = velocity;
        this.acceleration = acceleration;
        this.jerk = jerk;
    }
    public MotionState(MotionState oldMotionState){
        this.distance = oldMotionState.distance;
        this.velocity = oldMotionState.velocity;
        this.acceleration = oldMotionState.acceleration;
        this.jerk = oldMotionState.jerk;
    }
}
