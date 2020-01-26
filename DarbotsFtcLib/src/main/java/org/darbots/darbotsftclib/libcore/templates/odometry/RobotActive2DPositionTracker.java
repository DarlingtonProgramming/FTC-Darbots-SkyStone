package org.darbots.darbotsftclib.libcore.templates.odometry;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPose2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotVector2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.XYPlaneCalculations;
import org.darbots.darbotsftclib.libcore.runtime.GlobalRegister;
import org.darbots.darbotsftclib.libcore.templates.RobotNonBlockingDevice;
import org.darbots.darbotsftclib.libcore.templates.other_sensors.RobotGyro;

//Factor = trackedDistance / actualDistance
public abstract class RobotActive2DPositionTracker extends RobotSynchronized2DPositionTracker implements RobotGyro {
    private class RobotActive2DPositionTracker_Runnable implements Runnable{
        private volatile boolean m_RunningCommand = false;
        private volatile boolean m_RunningFlag = false;
        private ElapsedTime m_Time = null;

        @Override
        public void run() {
            m_RunningFlag = true;
            m_Time = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

            __trackStart();

            m_Time.reset();


            while(this.m_RunningCommand){
                try{
                    Thread.sleep(RobotActive2DPositionTracker.this.m_ThreadSleepTimeInMs);
                }catch(InterruptedException e){
                    e.printStackTrace();
                }

                double time = m_Time.seconds();
                m_Time.reset();
                __trackLoop(time);
                if(GlobalRegister.runningOpMode != null && GlobalRegister.runningOpMode.isStopRequested()){
                    break;
                }
            }
            m_RunningFlag = false;
            m_RunningCommand = false;
        }

        public void stop(){
            this.m_RunningCommand = false;
            while(this.m_RunningFlag){
                try {
                    Thread.sleep(20);
                }catch(InterruptedException e){

                }
            }
        }

        public boolean isRunning(){
            return this.m_RunningCommand || this.m_RunningFlag;
        }
    }
    private RobotActive2DPositionTracker_Runnable m_RunnableTracking = null;
    private Thread m_TrackingThread = null;
    private RobotGyro m_GyroProvider;
    private float m_LastGyroReading;
    private float m_GyroReadingAtZero;

    protected volatile double m_XDistanceFactor, m_YDistanceFactor, m_ZRotDistanceFactor;
    protected volatile int m_ThreadSleepTimeInMs = 20;
    public RobotActive2DPositionTracker(RobotPose2D initialPosition) {
        super(initialPosition);
        this.m_XDistanceFactor = 1;
        this.m_YDistanceFactor = 1;
        this.m_ZRotDistanceFactor = 1;
        this.__setupRunnableTracking();
    }

    public RobotActive2DPositionTracker(Robot2DPositionTracker oldTracker) {
        super(oldTracker);
        this.m_XDistanceFactor = 1;
        this.m_YDistanceFactor = 1;
        this.m_ZRotDistanceFactor = 1;
        this.__setupRunnableTracking();
    }

    public RobotActive2DPositionTracker(RobotActive2DPositionTracker oldTracker){
        super(oldTracker);
        RobotVector2D oldDistanceFactor = oldTracker.getDistanceFactors();
        this.m_XDistanceFactor = oldDistanceFactor.X;
        this.m_YDistanceFactor = oldDistanceFactor.Y;
        this.m_ZRotDistanceFactor = oldDistanceFactor.getRotationZ();
        this.setGyroProvider(oldTracker.m_GyroProvider);
        this.__setupRunnableTracking();
    }

    protected void __setupRunnableTracking(){
        this.m_RunnableTracking = new RobotActive2DPositionTracker_Runnable();
    }

    public RobotVector2D getDistanceFactors(){
        return new RobotVector2D(this.m_XDistanceFactor,this.m_YDistanceFactor,this.m_ZRotDistanceFactor);
    }

    /**
     * Set X,Y,ZRot Distance Factors to the Active Position Tracker
     * deltaX,Y,Z = trackedDeltaX,Y,Z / distanceFactor
     * @param DistanceFactor The distance factors of each axis / rotation
     */
    public void setDistanceFactors(RobotVector2D DistanceFactor){
        this.m_XDistanceFactor = DistanceFactor.X;
        this.m_YDistanceFactor = DistanceFactor.Y;
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

    protected void drive_MoveThroughRobotAxisOffset(RobotPose2D robotAxisValues) {
        synchronized (super.m_CurrentPos) {
            RobotPose2D tempField = XYPlaneCalculations.getAbsolutePosition(super.m_CurrentPos,robotAxisValues);
            super.m_CurrentPos = tempField;
        }
    }

    protected void __trackLoopMoved(RobotVector2D velocity, RobotPose2D deltaRobotAxis){
        RobotVector2D fixedVelocity = new RobotVector2D(velocity.X / this.m_XDistanceFactor,velocity.Y / this.m_YDistanceFactor,velocity.getRotationZ() / this.m_ZRotDistanceFactor);
        RobotPose2D fixedDeltaRobotAxis = new RobotPose2D(deltaRobotAxis.X / this.m_XDistanceFactor, deltaRobotAxis.Y / this.m_YDistanceFactor, deltaRobotAxis.getRotationZ() / this.m_ZRotDistanceFactor);
        this.__trackLoopMovedRaw(fixedVelocity,fixedDeltaRobotAxis);
    }

    protected void __trackLoopMovedRaw(RobotVector2D velocity, RobotPose2D deltaRobotAxis){
        this.setCurrentVelocityVector(velocity);
        this.drive_MoveThroughRobotAxisOffset(deltaRobotAxis);
    }

    protected abstract void __trackStart();
    protected abstract void __trackLoop(double secondsSinceLastLoop);

    @Override
    public void stop(){
        this.m_RunnableTracking.stop();
    }
    public void start(){
        if(this.m_RunnableTracking.isRunning()){
            return;
        }
        this.m_TrackingThread = new Thread(this.m_RunnableTracking);
        this.m_RunnableTracking.m_RunningCommand = true;
        this.m_TrackingThread.start();
    }
    public boolean isRunning(){
        return this.m_RunnableTracking.isRunning();
    }
    @Override
    public float getHeading() {
        RobotPose2D currentPose = this.getCurrentPosition();
        return (float) currentPose.getRotationZ();
    }

    @Override
    public HeadingRotationPositiveOrientation getHeadingRotationPositiveOrientation() {
        return HeadingRotationPositiveOrientation.CounterClockwise;
    }

    public RobotGyro getGyroProvider(){
        return this.m_GyroProvider;
    }

    public void setGyroProvider(RobotGyro provider){
        this.m_GyroProvider = provider;
        if(provider != null) {
            this.updateGyroProvider();
            this.m_LastGyroReading = this.m_GyroProvider.getHeading();
            RobotPose2D currentPosition = super.getCurrentPosition();
            this.m_GyroReadingAtZero = this.m_LastGyroReading - ((float) currentPosition.getRotationZ());
        }
    }

    protected void updateGyroProvider(){
        if(this.m_GyroProvider != null && this.m_GyroProvider instanceof RobotNonBlockingDevice){
            ((RobotNonBlockingDevice) this.m_GyroProvider).updateStatus();
        }
    }

    protected double getDeltaAng(double supposedDeltaAng){
        if(this.m_GyroProvider == null){
            return supposedDeltaAng;
        }else{
            this.updateGyroProvider();
            float newAng = this.m_GyroProvider.getHeading();
            float deltaAngGyro = XYPlaneCalculations.normalizeDeg(newAng - this.m_LastGyroReading);
            if(this.m_GyroProvider.getHeadingRotationPositiveOrientation() == HeadingRotationPositiveOrientation.Clockwise){
                deltaAngGyro = -deltaAngGyro;
            }
            this.m_LastGyroReading = newAng;
            return deltaAngGyro;
        }
    }

    public RobotPose2D getCurrentPosition(){
        RobotPose2D currentPose = super.getCurrentPosition();
        if(this.m_GyroProvider != null){
            double currentGyroReading = this.m_GyroProvider.getHeading();
            double deltaAng = currentGyroReading - this.m_GyroReadingAtZero;
            if(this.m_GyroProvider.getHeadingRotationPositiveOrientation() == HeadingRotationPositiveOrientation.Clockwise){
                deltaAng = -deltaAng;
            }
            currentPose.setRotationZ(XYPlaneCalculations.normalizeDeg(deltaAng));
        }
        return currentPose;
    }

    public void setCurrentPosition(RobotPose2D currentPosition){
        super.setCurrentPosition(currentPosition);
        if(this.m_GyroProvider != null) {
            this.updateGyroProvider();
            this.m_LastGyroReading = this.m_GyroProvider.getHeading();
            this.m_GyroReadingAtZero = this.m_LastGyroReading - ((float) currentPosition.getRotationZ());
        }
    }
}
