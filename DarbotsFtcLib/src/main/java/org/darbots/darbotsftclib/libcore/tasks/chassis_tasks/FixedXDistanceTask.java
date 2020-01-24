package org.darbots.darbotsftclib.libcore.tasks.chassis_tasks;

import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPoint2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPose2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotVector2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.XYPlaneCalculations;
import org.darbots.darbotsftclib.libcore.templates.chassis_related.RobotMotionSystemTask;

public class FixedXDistanceTask extends RobotMotionSystemTask {
    private double distance;
    private double startSpeed;
    private double endSpeed;
    private double slowDownFactor;
    private double normalizedSpeed;
    private double preferredAngle;

    public FixedXDistanceTask(double XDistance, double SlowDownFactor, double StartSpeed, double CruiseSpeed, double EndSpeed, double PreferredAngle){
        this.setDistance(XDistance);
        this.setSlowDownFactor(SlowDownFactor);
        this.setStartSpeed(StartSpeed);
        this.setCruiseSpeed(CruiseSpeed);
        this.setEndSpeed(EndSpeed);
        this.setPreferredAngle(PreferredAngle);
    }

    public FixedXDistanceTask(FixedXDistanceTask oldTask){

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
        this.getMotionSystem().setRobotSpeed(startSpeed * this.getMotionSystem().calculateMaxLinearXSpeedInCMPerSec(),0,0);
    }

    @Override
    protected void __taskFinished() {

    }

    @Override
    protected void __updateStatus() {
        RobotPose2D currentOffset = this.getRelativePositionOffsetSinceStart();
        RobotPoint2D currentTarget = XYPlaneCalculations.getRelativePosition(currentOffset,new RobotPoint2D(distance,0));

        double yError = 0 - currentOffset.Y;
        double angleError = XYPlaneCalculations.normalizeDeg(preferredAngle - currentOffset.getRotationZ());
        RobotVector2D errorCorection = this.getErrorCorrectionVelocityVector(currentOffset,0,yError,angleError);
        double xTravelled = currentOffset.X;
        if(distance >= 0 && Math.abs(xTravelled) >= Math.abs(distance)){
            this.stopTask();
            return;
        }else if(distance < 0 && Math.abs(xTravelled) >= Math.abs(distance)){
            this.stopTask();
            return;
        }
        double progress = xTravelled / distance;
        double normalizedXSpeed;
        if(progress < slowDownFactor){
            normalizedXSpeed = (normalizedSpeed - startSpeed) * progress + startSpeed;
        }else if(progress > (1.0-slowDownFactor)){
            normalizedXSpeed = (endSpeed - normalizedSpeed) * (1.0-progress) + normalizedSpeed;
        }else{
            normalizedXSpeed = normalizedSpeed;
        }
        if(this.distance < 0){
            normalizedXSpeed = -normalizedXSpeed;
        }
        double temporarySpeed = Math.sqrt(Math.pow(currentTarget.X,2) + Math.pow(currentTarget.Y,2));
        double temporaryScale = normalizedXSpeed * temporarySpeed;
        double scaledX = currentTarget.X * temporaryScale;
        double scaledY = currentTarget.Y * temporaryScale;
        RobotPoint2D errorCorrectionRaw = errorCorection.toPoint2D();
        RobotPoint2D errorCorrectionVal = XYPlaneCalculations.getRelativePosition(new RobotPose2D(0,0,currentOffset.getRotationZ()),errorCorrectionRaw);
        this.getMotionSystem().setRobotSpeed(errorCorrectionVal.X + scaledX * this.getMotionSystem().calculateMaxLinearXSpeedInCMPerSec(),errorCorrectionVal.Y + scaledY * this.getMotionSystem().calculateMaxLinearYSpeedInCMPerSec(),errorCorection.getRotationZ());
    }

    @Override
    protected RobotPose2D __getSupposedTaskFinishPos() {
        return new RobotPose2D(distance,0,preferredAngle);
    }
}
