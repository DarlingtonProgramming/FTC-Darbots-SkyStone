package org.darbots.darbotsftclib.libcore_4_5_0Pre.tasks.chassis_tasks;

import org.darbots.darbotsftclib.libcore_4_5_0Pre.calculations.dimentional_calculation.RobotPoint2D;
import org.darbots.darbotsftclib.libcore_4_5_0Pre.calculations.dimentional_calculation.RobotPose2D;
import org.darbots.darbotsftclib.libcore_4_5_0Pre.calculations.dimentional_calculation.RobotVector2D;
import org.darbots.darbotsftclib.libcore_4_5_0Pre.calculations.dimentional_calculation.XYPlaneCalculations;
import org.darbots.darbotsftclib.libcore_4_5_0Pre.templates.chassis_related.RobotMotionSystemTask;

public class FixedYDistanceTask extends RobotMotionSystemTask {
    private double distance;
    private double startSpeed;
    private double endSpeed;
    private double slowDownFactor;
    private double normalizedSpeed;
    private double preferredAngle;

    public FixedYDistanceTask(double YDistance, double SlowDownFactor, double StartSpeed, double CruiseSpeed, double EndSpeed, double PreferredAngle){
        this.setDistance(YDistance);
        this.setSlowDownFactor(SlowDownFactor);
        this.setStartSpeed(StartSpeed);
        this.setCruiseSpeed(CruiseSpeed);
        this.setEndSpeed(EndSpeed);
        this.setPreferredAngle(PreferredAngle);
    }

    public FixedYDistanceTask(FixedXDistanceTask oldTask){

    }

    public double getDistance(){
        return this.distance;
    }

    public void setDistance(double distance){
        this.distance = distance;
    }

    public double getStartSpeed(){
        return this.startSpeed;
    }

    public void setStartSpeed(double startSpeed){
        this.startSpeed = Math.abs(startSpeed);
    }

    public double getEndSpeed(){
        return this.endSpeed;
    }

    public void setEndSpeed(double endSpeed){
        this.endSpeed = Math.abs(endSpeed);
    }

    public double getSlowDownFactor(){
        return this.slowDownFactor;
    }

    public void setSlowDownFactor(double slowDownFactor){
        this.slowDownFactor = Math.abs(slowDownFactor);
    }

    public double getCruiseSpeed(){
        return this.normalizedSpeed;
    }

    public void setCruiseSpeed(double cruiseSpeed){
        this.normalizedSpeed = Math.abs(cruiseSpeed);
    }

    public double getPreferredAngle(){
        return this.startSpeed;
    }

    public void setPreferredAngle(double preferredAngle){
        this.preferredAngle = XYPlaneCalculations.normalizeDeg(preferredAngle);
    }

    @Override
    protected void __startTask() {
        double realStartSpeed = startSpeed * this.getMotionSystem().calculateMaxLinearXSpeedInCMPerSec();
        if(this.distance < 0){
            realStartSpeed = -realStartSpeed;
        }
        this.getMotionSystem().setRobotSpeed(0,realStartSpeed,0);
    }

    @Override
    protected void __taskFinished() {

    }

    @Override
    protected void __updateStatus() {
        RobotPose2D currentOffset = this.getRelativePositionOffsetSinceStart();
        RobotPoint2D currentTarget = XYPlaneCalculations.getRelativePosition(currentOffset,new RobotPoint2D(0,distance));

        double xError = 0 - currentOffset.X;
        double angleError = XYPlaneCalculations.normalizeDeg(preferredAngle - currentOffset.getRotationZ());
        RobotVector2D errorCorrection = this.getErrorCorrectionVelocityVector(currentOffset,xError,0,angleError);
        double yTravelled = currentOffset.Y;
        if(distance >= 0 && Math.abs(yTravelled) >= Math.abs(distance)){
            this.stopTask();
            return;
        }else if(distance < 0 && Math.abs(yTravelled) >= Math.abs(distance)){
            this.stopTask();
            return;
        }
        double progress = yTravelled / distance;
        double normalizedYSpeed;
        if(progress < slowDownFactor){
            normalizedYSpeed = (normalizedSpeed - startSpeed) * progress + startSpeed;
        }else if(progress > (1.0-slowDownFactor)){
            normalizedYSpeed = (endSpeed - normalizedSpeed) * (progress - (1.0 - slowDownFactor)) + normalizedSpeed;
        }else{
            normalizedYSpeed = normalizedSpeed;
        }
        double temporarySpeed = Math.sqrt(Math.pow(currentTarget.X,2) + Math.pow(currentTarget.Y,2));
        double temporaryScale = Math.abs(normalizedYSpeed * temporarySpeed);
        double scaledX = currentTarget.X * temporaryScale;
        double scaledY = currentTarget.Y * temporaryScale;
        RobotPoint2D errorCorrectionRaw = errorCorrection.toPoint2D();
        RobotPoint2D errorCorrectionVal = XYPlaneCalculations.getRelativePosition(new RobotPose2D(0,0,currentOffset.getRotationZ()),errorCorrectionRaw);
        this.getMotionSystem().setRobotSpeed(errorCorrectionVal.X + scaledX * this.getMotionSystem().calculateMaxLinearXSpeedInCMPerSec(),errorCorrectionVal.Y + scaledY * this.getMotionSystem().calculateMaxLinearYSpeedInCMPerSec(),errorCorrection.getRotationZ());
    }

    @Override
    protected RobotPose2D __getSupposedTaskFinishPos() {
        return new RobotPose2D(distance,0,preferredAngle);
    }
}
