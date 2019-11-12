package org.darbots.darbotsftclib.libcore.odometry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.darbots.darbotsftclib.libcore.calculations.dimentionalcalculation.Robot2DPositionIndicator;
import org.darbots.darbotsftclib.libcore.sensors.gyros.SynchronizedSoftwareGyro;
import org.darbots.darbotsftclib.libcore.sensors.motion_related.RobotMotion;
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

                double deltaLTCM = deltaLTCount / this.;
                double deltaRTCM = deltaLeftCount / this.m_LeftEncoderCountsPerCM;
                double deltaRightCM = deltaRightCount / this.m_RightEncoderCountsPerCM;

                double deltaXMoved = (-deltaLeftCM + deltaRightCM) / 2;
                double deltaYMoved = deltaMidCM;
                double deltaAngMoved = (deltaLeftCM / this.m_LeftEncoderRotationCircumferenceInCM + deltaRightCM / this.m_LeftEncoderRotationCircumferenceInCM) / 2.0 * 360.0;

                Robot2DPositionIndicator currentVelocityVector = new Robot2DPositionIndicator(
                        deltaXMoved / secondsDriven,
                        deltaYMoved / secondsDriven,
                        deltaAngMoved / secondsDriven
                );

                Robot3Wheel2DTracker.this.drive_MoveThroughRobotAxisOffset(new Robot2DPositionIndicator(
                        deltaXMoved,
                        deltaYMoved,
                        deltaAngMoved
                ));

                Robot3Wheel2DTracker.this.setCurrentVelocityVector(currentVelocityVector);

                if(this.m_Gyro != null){
                    this.m_Gyro.offsetHeading((float) deltaAngMoved);
                }

                m_LastMidEncoderCount = newMidCount;
                m_LastRightEncoderCount = newRightCount;
                m_LastLeftEncoderCount = newLeftCount;

            }
            m_IsRunning = false;
        }

        public void stop(){
            this.m_IsRunning = false;
        }
    }

    private double m_LeftEncoderCountsPerRev = 0;
    private double m_RightEncoderCountsPerRev = 0;
    private double m_MidEncoderCountsPerRev = 0;

    private double m_LeftEncoderWheelCircumference = 0;
    private double m_RightEncoderWheelCircumference = 0;
    private double m_MidEncoderWheelCircumference = 0;

    private Robot3Wheel2DTracker.Robot2DPassive3WheelTracker_Runnable m_RunnableTracking = null;
    private Thread m_TrackingThread = null;
    private boolean m_TrackingThreadRunned = false;


    public Robot3Wheel2DTracker(Robot2DPositionIndicator initialPosition, boolean initSoftwareGyro, DcMotor leftEncoder, DcMotor rightEncoder, DcMotor centerEncoder, double LeftEncoderCountsPerRev, double LeftEncoderWheelRadius, double LeftEncoderDistanceFromCenterOfRobot, double RightEncoderCountsPerRev, double RightEncoderWheelRadius, double RightEncoderDistanceFromCenterOfRobot, double MidEncoderCountsPerRev, double MidEncoderWheelRadius) {
        super(initialPosition);
        __setupRunnable();
        __setupParams(initSoftwareGyro,leftEncoder,rightEncoder,centerEncoder,LeftEncoderCountsPerRev,LeftEncoderWheelRadius,LeftEncoderDistanceFromCenterOfRobot,RightEncoderCountsPerRev,RightEncoderWheelRadius,RightEncoderDistanceFromCenterOfRobot,MidEncoderCountsPerRev,MidEncoderWheelRadius);
    }

    public Robot3Wheel2DTracker(Robot3Wheel2DTracker oldTracker) {
        super(oldTracker);
        __setupRunnable();
        __setupParams(oldTracker.getGyro() != null,oldTracker.getLeftEncoder(),oldTracker.getRightEncoder(),oldTracker.getMidEncoder(),oldTracker.getLeftEncoderCountsPerRev(),oldTracker.getLeftEncoderWheelRadius(),oldTracker.getDistanceOfLeftEncoderFromCenterOfRobot(),oldTracker.getRightEncoderCountsPerRev(),oldTracker.getRightEncoderWheelRadius(),oldTracker.getDistanceOfRightEncoderFromCenterOfRobot(),oldTracker.getMidEncoderCountsPerRev(),oldTracker.getMidEncoderWheelRadius());
    }

    private void __setupRunnable(){
        this.m_RunnableTracking = new Robot3Wheel2DTracker.Robot2DPassive3WheelTracker_Runnable();
        this.m_TrackingThread = new Thread(this.m_RunnableTracking);
        this.m_TrackingThreadRunned = false;
    }

    private void __setupParams(boolean initSoftwareGyro, DcMotor leftEncoder, DcMotor rightEncoder, DcMotor centerEncoder, double LeftEncoderCountsPerRev, double LeftEncoderWheelRadius, double LeftEncoderDistanceFromCenterOfRobot, double RightEncoderCountsPerRev, double RightEncoderWheelRadius, double RightEncoderDistanceFromCenterOfRobot, double MidEncoderCountsPerRev, double MidEncoderWheelRadius) {
        this.m_RunnableTracking.m_LeftEncoder = leftEncoder;
        this.m_RunnableTracking.m_RightEncoder = rightEncoder;
        this.m_RunnableTracking.m_MidEncoder = centerEncoder;

        this.m_LeftEncoderCountsPerRev = LeftEncoderCountsPerRev;
        this.m_LeftEncoderWheelCircumference = LeftEncoderWheelRadius * (2 * Math.PI);
        this.__recalculateLeftCountsPerCM();

        this.m_MidEncoderCountsPerRev = MidEncoderCountsPerRev;
        this.m_MidEncoderWheelCircumference = MidEncoderWheelRadius * (2 * Math.PI);
        this.__recalculateMidCountsPerCM();

        this.m_RightEncoderCountsPerRev = RightEncoderCountsPerRev;
        this.m_RightEncoderWheelCircumference = RightEncoderWheelRadius * (2 * Math.PI);
        this.__recalculateRightCountsPerCM();

        this.setDistanceOfLeftEncoderFromCenterOfRobot(LeftEncoderDistanceFromCenterOfRobot);
        this.setDistanceOfRightEncoderFromCenterOfRobot(RightEncoderDistanceFromCenterOfRobot);

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

    public DcMotor getLeftEncoder(){
        return this.m_RunnableTracking.m_LeftEncoder;
    }

    public DcMotor getRightEncoder(){
        return this.m_RunnableTracking.m_RightEncoder;
    }

    public DcMotor getMidEncoder(){
        return this.m_RunnableTracking.m_MidEncoder;
    }

    public double getDistanceOfLeftEncoderFromCenterOfRobot(){
        return this.m_RunnableTracking.m_LeftEncoderRotationCircumferenceInCM / (2 * Math.PI);
    }

    public void setDistanceOfLeftEncoderFromCenterOfRobot(double Distance){
        this.m_RunnableTracking.m_LeftEncoderRotationCircumferenceInCM = Distance * (2 * Math.PI);
    }

    public double getDistanceOfRightEncoderFromCenterOfRobot(){
        return this.m_RunnableTracking.m_RightEncoderRotationCircumferenceInCM / (2 * Math.PI);
    }

    public void setDistanceOfRightEncoderFromCenterOfRobot(double Distance){
        this.m_RunnableTracking.m_RightEncoderRotationCircumferenceInCM = Distance * (2 * Math.PI);
    }

    public boolean isLeftEncoderReversed(){
        return this.m_RunnableTracking.m_LeftEncoderReversed;
    }

    public void setLeftEncoderReversed(boolean Reversed){
        this.m_RunnableTracking.m_LeftEncoderReversed = Reversed;
    }

    public boolean isRightEncoderReversed(){
        return this.m_RunnableTracking.m_RightEncoderReversed;
    }

    public void setRightEncoderReversed(boolean Reversed){
        this.m_RunnableTracking.m_RightEncoderReversed = Reversed;
    }

    public boolean isMidEncoderReversed(){
        return this.m_RunnableTracking.m_MidEncoderReversed;
    }

    public void setMidEncoderReversed(boolean Reversed){
        this.m_RunnableTracking.m_MidEncoderReversed = Reversed;
    }

    public double getLeftEncoderCountsPerRev(){
        return this.m_LeftEncoderCountsPerRev;
    }

    public void setLeftEncoderCountsPerRev(double CountsPerRev){
        this.m_LeftEncoderCountsPerRev = CountsPerRev;
        this.__recalculateLeftCountsPerCM();
    }

    public double getLeftEncoderWheelCircumference(){
        return this.m_LeftEncoderWheelCircumference;
    }

    public double getLeftEncoderWheelRadius(){
        return this.m_LeftEncoderWheelCircumference / (2 * Math.PI);
    }

    public void setLeftEncoderWheelCircumference(double Circumference){
        this.m_LeftEncoderWheelCircumference = Circumference;
        this.__recalculateLeftCountsPerCM();
    }

    public void setLeftEncoderWheelRadius(double Radius){
        this.setLeftEncoderWheelCircumference(Radius * (2 * Math.PI));
    }

    protected void __recalculateLeftCountsPerCM(){
        this.m_RunnableTracking.m_LeftEncoderCountsPerCM = this.m_LeftEncoderCountsPerRev * (1.0 / this.m_LeftEncoderWheelCircumference);
    }
    public double getRightEncoderCountsPerRev(){
        return this.m_RightEncoderCountsPerRev;
    }

    public void setRightEncoderCountsPerRev(double CountsPerRev){
        this.m_RightEncoderCountsPerRev = CountsPerRev;
        this.__recalculateRightCountsPerCM();
    }

    public double getRightEncoderWheelCircumference(){
        return this.m_RightEncoderWheelCircumference;
    }

    public double getRightEncoderWheelRadius(){
        return this.m_RightEncoderWheelCircumference / (2 * Math.PI);
    }

    public void setRightEncoderWheelCircumference(double Circumference){
        this.m_RightEncoderWheelCircumference = Circumference;
        this.__recalculateRightCountsPerCM();
    }

    public void setRightEncoderWheelRadius(double Radius){
        this.setRightEncoderWheelCircumference(Radius * (2 * Math.PI));
    }

    protected void __recalculateRightCountsPerCM(){
        this.m_RunnableTracking.m_RightEncoderCountsPerCM = this.m_RightEncoderCountsPerRev * (1.0 / this.m_RightEncoderWheelCircumference);
    }

    public double getMidEncoderCountsPerRev(){
        return this.m_MidEncoderCountsPerRev;
    }

    public void setMidEncoderCountsPerRev(double CountsPerRev){
        this.m_MidEncoderCountsPerRev = CountsPerRev;
        this.__recalculateMidCountsPerCM();
    }

    public double getMidEncoderWheelCircumference(){
        return this.m_MidEncoderWheelCircumference;
    }

    public double getMidEncoderWheelRadius(){
        return this.m_MidEncoderWheelCircumference / (2 * Math.PI);
    }

    public void setMidEncoderWheelCircumference(double Circumference){
        this.m_MidEncoderWheelCircumference = Circumference;
        this.__recalculateMidCountsPerCM();
    }

    public void setMidEncoderWheelRadius(double Radius){
        this.setMidEncoderWheelCircumference(Radius * (2 * Math.PI));
    }

    protected void __recalculateMidCountsPerCM(){
        this.m_RunnableTracking.m_MidEncoderCountsPerCM = this.m_MidEncoderCountsPerRev * (1.0 / this.m_MidEncoderWheelCircumference);
    }

    protected void drive_MoveThroughRobotAxisOffset(Robot2DPositionIndicator robotAxisValues) {
        Robot2DPositionIndicator tempField = this.fieldAxisFromRobotAxis(robotAxisValues);
        this.setCurrentPosition(tempField);
        this.offsetRelative(robotAxisValues);
    }
}
