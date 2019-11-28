package org.darbots.darbotsftclib.libcore.templates.chassis_related;

public class MotionSystemConstraints {
    public double maximumLinearSpeed;
    public double maximumLinearAcceleration;
    public double maximumLinearJerk;
    public MotionSystemConstraints(double maximumLinearSpeed, double maximumLinearAcceleration, double maximumLinearJerk){
        this.maximumLinearSpeed = maximumLinearSpeed;
        this.maximumLinearAcceleration = maximumLinearAcceleration;
        this.maximumLinearJerk = maximumLinearJerk;
    }
    public MotionSystemConstraints(MotionSystemConstraints oldConstraint){
        this.maximumLinearSpeed = oldConstraint.maximumLinearSpeed;
        this.maximumLinearAcceleration = oldConstraint.maximumLinearAcceleration;
        this.maximumLinearJerk = oldConstraint.maximumLinearJerk;
    }
}
