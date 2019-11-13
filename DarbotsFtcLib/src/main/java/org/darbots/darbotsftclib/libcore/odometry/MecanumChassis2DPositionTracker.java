package org.darbots.darbotsftclib.libcore.odometry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.darbots.darbotsftclib.libcore.calculations.dimentionalcalculation.Robot2DPositionIndicator;
import org.darbots.darbotsftclib.libcore.calculations.dimentionalcalculation.XYPlaneCalculations;
import org.darbots.darbotsftclib.libcore.sensors.gyros.SynchronizedSoftwareGyro;
import org.darbots.darbotsftclib.libcore.sensors.motion_related.RobotMotion;
import org.darbots.darbotsftclib.libcore.sensors.motion_related.RobotWheel;
import org.darbots.darbotsftclib.libcore.templates.motor_related.RobotMotor;
import org.darbots.darbotsftclib.libcore.templates.odometry.RobotSynchronized2DPositionTracker;

public class MecanumChassis2DPositionTracker extends RobotSynchronized2DPositionTracker {
    private class MecanumChassis2DPositionTracker_Runnable implements Runnable{
        private volatile int m_SleepTimeInMillis = 75;
        private volatile boolean m_IsRunning = false;

        private SynchronizedSoftwareGyro m_Gyro = null;

        private RobotMotion LTMotion, RTMotion, LBMotion, RBMotion;

        private int m_LastLTEncoderCount = 0;
        private int m_LastRTEncoderCount = 0;
        private int m_LastLBEncoderCount = 0;
        private int m_LastRBEncoderCount = 0;

        private double m_CONST_LTCountsPerDeg = 0;
        private double m_CONST_RTCountsPerDeg = 0;
        private double m_CONST_LBCountsPerDeg = 0;
        private double m_CONST_RBCountsPerDeg = 0;

        private double m_CONST_180_OVER_PIR = 0;
        private double m_CONST_PIR_OVER_4T180 = 0;
        private double m_CONST_R_OVER_4TKl = 0;

        private ElapsedTime m_Time = null;

        public boolean isRunning(){
            return this.m_IsRunning;
        }

        @Override
        public void run() {
            m_IsRunning = true;

            m_Time = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

            RobotMotor LTMotor = LTMotion.getMotorController().getMotor(),
                    RTMotor = RTMotion.getMotorController().getMotor(),
                    LBMotor = LBMotion.getMotorController().getMotor(),
                    RBMotor = RBMotion.getMotorController().getMotor();


            m_LastLTEncoderCount = LTMotor.getCurrentCount();
            m_LastRTEncoderCount = RTMotor.getCurrentCount();
            m_LastLBEncoderCount = LBMotor.getCurrentCount();
            m_LastRBEncoderCount = RBMotor.getCurrentCount();

            m_Time.reset();

            while(m_IsRunning){
                try{
                    Thread.sleep(m_SleepTimeInMillis);
                }catch(InterruptedException e){
                    e.printStackTrace();
                }

                int newLTCount = LTMotor.getCurrentCount();
                int newRTCount = RTMotor.getCurrentCount();
                int newLBCount = LBMotor.getCurrentCount();
                int newRBCount = RBMotor.getCurrentCount();

                double secondsDriven = m_Time.seconds();

                int deltaLTCount = newLTCount - m_LastLTEncoderCount;
                int deltaRTCount = newRTCount - m_LastRTEncoderCount;
                int deltaLBCount = newLBCount - m_LastLBEncoderCount;
                int deltaRBCount = newRBCount - m_LastRBEncoderCount;

                double deltaLTDeg = deltaLTCount / this.m_CONST_LTCountsPerDeg;
                double deltaRTDeg = deltaRTCount / this.m_CONST_RTCountsPerDeg;
                double deltaLBDeg = deltaLBCount / this.m_CONST_LBCountsPerDeg;
                double deltaRBDeg = deltaRBCount / this.m_CONST_RBCountsPerDeg;

                double deltaLTSpeedInDegPerSec = deltaLTDeg / secondsDriven;
                double deltaRTSpeedInDegPerSec = deltaRTDeg / secondsDriven;
                double deltaLBSpeedInDegPerSec = deltaLBDeg / secondsDriven;
                double deltaRBSpeedInDegPerSec = deltaRBDeg / secondsDriven;

                /*
                Here we are not calculating the Speed of the Robot by using our static method
                This is because the calculation of some constants takes up a lot of time, and we don't want to waste our time on repeatedly calculating those constants.
                Those constants are calculated when we are initializing the MecanumChassis2DPositionTrackerClass, and they help us to reduce double multiplications in speed calculation.
                 */



                double deltaXMoved = 0;
                double deltaYMoved = 0;
                double deltaAngMoved = 0;

                Robot2DPositionIndicator currentVelocityVector = new Robot2DPositionIndicator(
                        deltaXMoved / secondsDriven,
                        deltaYMoved / secondsDriven,
                        deltaAngMoved / secondsDriven
                );

                MecanumChassis2DPositionTracker.this.drive_MoveThroughRobotAxisOffset(new Robot2DPositionIndicator(
                        deltaXMoved,
                        deltaYMoved,
                        deltaAngMoved
                ));

                MecanumChassis2DPositionTracker.this.setCurrentVelocityVector(currentVelocityVector);

                if(this.m_Gyro != null){
                    this.m_Gyro.offsetHeading((float) deltaAngMoved);
                }

                m_LastLTEncoderCount = newLTCount;
                m_LastRTEncoderCount = newRTCount;
                m_LastLBEncoderCount = newLBCount;
                m_LastRBEncoderCount = newRBCount;

            }
            m_IsRunning = false;
        }

        public void stop(){
            this.m_IsRunning = false;
        }
    }

    private MecanumChassis2DPositionTracker_Runnable m_RunnableTracking = null;
    private Thread m_TrackingThread = null;
    private boolean m_TrackingThreadRunned = false;


    public MecanumChassis2DPositionTracker(Robot2DPositionIndicator initialPosition, boolean initSoftwareGyro, RobotMotion LTMotion, RobotMotion RTMotion, RobotMotion LBMotion, RobotMotion RBMotion){
        super(initialPosition);
        __setupRunnable();
        __setupParams(initSoftwareGyro, LTMotion, RTMotion, LBMotion, RBMotion);
    }

    public MecanumChassis2DPositionTracker(MecanumChassis2DPositionTracker oldTracker) {
        super(oldTracker);
        __setupRunnable();
        __setupParams(oldTracker.getGyro() != null, oldTracker.getLTMotion(), oldTracker.getRTMotion(), oldTracker.getLBMotion(), oldTracker.getRBMotion());
    }

    private void __setupRunnable(){
        this.m_RunnableTracking = new MecanumChassis2DPositionTracker_Runnable();
        this.m_TrackingThread = new Thread(this.m_RunnableTracking);
        this.m_TrackingThreadRunned = false;
    }

    private void __setupParams(boolean initSoftwareGyro, RobotMotion LTMotion, RobotMotion RTMotion, RobotMotion LBMotion, RobotMotion RBMotion) {
        this.m_RunnableTracking.LTMotion = LTMotion;
        this.m_RunnableTracking.RTMotion = RTMotion;
        this.m_RunnableTracking.LBMotion = LBMotion;
        this.m_RunnableTracking.RBMotion = RBMotion;

        this.m_RunnableTracking.m_CONST_180_OVER_PIR = XYPlaneCalculations.CONST_180_OVER_PI / LTMotion.getRobotWheel().getRadius();
        this.m_RunnableTracking.m_CONST_PIR_OVER_4T180 = XYPlaneCalculations.CONST_PI_OVER_180 * LTMotion.getRobotWheel().getRadius() / 4.0;
        this.m_RunnableTracking.m_CONST_R_OVER_4TKl = LTMotion.getRobotWheel().getRadius() / (4 * (LTMotion.getRobotWheel().getOnRobotPosition().getX() + LTMotion.getRobotWheel().getOnRobotPosition().getY()));

        this.m_RunnableTracking.m_CONST_LTCountsPerDeg = LTMotion.getMotorController().getMotor().getMotorType().getCountsPerRev() / 360.0;
        this.m_RunnableTracking.m_CONST_RTCountsPerDeg = RTMotion.getMotorController().getMotor().getMotorType().getCountsPerRev() / 360.0;
        this.m_RunnableTracking.m_CONST_LBCountsPerDeg = LBMotion.getMotorController().getMotor().getMotorType().getCountsPerRev() / 360.0;
        this.m_RunnableTracking.m_CONST_RBCountsPerDeg = RBMotion.getMotorController().getMotor().getMotorType().getCountsPerRev() / 360.0;

        if(initSoftwareGyro){
            this.m_RunnableTracking.m_Gyro = new SynchronizedSoftwareGyro(0.0f);
        }
    }

    public float getSleepTimeInSec(){
        return this.m_RunnableTracking.m_SleepTimeInMillis / 1000.0f;
    }

    public void setSleepTimeInSec(float second){
        this.m_RunnableTracking.m_SleepTimeInMillis = Math.round(Math.abs(second) / 1000.0f);
    }

    public void stop(){
        this.m_RunnableTracking.stop();
    }

    public void start(){

        if(!m_TrackingThreadRunned) {
            this.m_TrackingThread.start();
            this.m_TrackingThreadRunned = true;
        }else{
            this.m_TrackingThread = new Thread(this.m_RunnableTracking);
            this.m_TrackingThread.start();
        }
    }

    public SynchronizedSoftwareGyro getGyro(){
        return this.m_RunnableTracking.m_Gyro;
    }

    public RobotMotion getLTMotion(){
        return this.m_RunnableTracking.LTMotion;
    }

    public RobotMotion getRTMotion(){
        return this.m_RunnableTracking.RTMotion;
    }

    public RobotMotion getLBMotion(){
        return this.m_RunnableTracking.LBMotion;
    }

    public RobotMotion getRBMotion(){
        return this.m_RunnableTracking.RBMotion;
    }

    protected void drive_MoveThroughRobotAxisOffset(Robot2DPositionIndicator robotAxisValues) {
        Robot2DPositionIndicator tempField = this.fieldAxisFromRobotAxis(robotAxisValues);
        this.setCurrentPosition(tempField);
        this.offsetRelative(robotAxisValues);
    }


    public static Robot2DPositionIndicator calculateRobotSpeed(double LTAngularSpeed, double RTAngularSpeed, double LBAngularSpeed, double RBAngularSpeed, RobotWheel LTWheel){

    }

    public static double[] calculateAngularSpeedInDegPerSecForEachWheel(Robot2DPositionIndicator RobotSpeed, RobotWheel LTWheel){

    }

}
