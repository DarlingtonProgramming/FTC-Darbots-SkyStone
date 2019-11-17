package org.darbots.darbotsftclib.libcore.templates.odometry;

import org.darbots.darbotsftclib.libcore.calculations.dimentionalcalculation.RobotPose2D;

public abstract class RobotActive2DPositionTracker extends RobotSynchronized2DPositionTracker {
    protected volatile double m_XDistanceFactor, m_YDistanceFactor, m_ZRotDistanceFactor;
    protected volatile int m_ThreadSleepTimeInMs = 75;
    public RobotActive2DPositionTracker(RobotPose2D initialPosition) {
        super(initialPosition);
        this.m_XDistanceFactor = 1;
        this.m_YDistanceFactor = 1;
        this.m_ZRotDistanceFactor = 1;
    }

    public RobotActive2DPositionTracker(Robot2DPositionTracker oldTracker) {
        super(oldTracker);
        this.m_XDistanceFactor = 1;
        this.m_YDistanceFactor = 1;
        this.m_ZRotDistanceFactor = 1;
    }

    public RobotActive2DPositionTracker(RobotActive2DPositionTracker oldTracker){
        super(oldTracker);
        RobotPose2D oldDistanceFactor = oldTracker.getDistanceFactors();
        this.m_XDistanceFactor = oldDistanceFactor.getX();
        this.m_YDistanceFactor = oldDistanceFactor.getY();
        this.m_ZRotDistanceFactor = oldDistanceFactor.getRotationZ();
    }

    public RobotPose2D getDistanceFactors(){
        return new RobotPose2D(this.m_XDistanceFactor,this.m_YDistanceFactor,this.m_ZRotDistanceFactor);
    }

    /**
     * Set X,Y,ZRot Distance Factors to the Active Position Tracker
     * deltaX,Y,Z = trackedDeltaX,Y,Z / distanceFactor
     * @param DistanceFactor The distance factors of each axis / rotation
     */
    public void setDistanceFactors(RobotPose2D DistanceFactor){
        this.m_XDistanceFactor = DistanceFactor.getX();
        this.m_YDistanceFactor = DistanceFactor.getY();
        this.m_ZRotDistanceFactor = DistanceFactor.getRotationZ();
    }

    public float getThreadSleepTimeInSec(){
        return this.m_ThreadSleepTimeInMs / 1000.0f;
    }

    public int getThreadSLeepTimeInMs(){
        return this.m_ThreadSleepTimeInMs;
    }

    public void setSleepTimeInSec(float second){
        this.m_ThreadSleepTimeInMs = Math.abs(Math.round(second / 1000.0f));
    }

    public void setThreadSleepTimeInMs(int milliSecond){
        this.m_ThreadSleepTimeInMs = Math.abs(milliSecond);
    }
}
