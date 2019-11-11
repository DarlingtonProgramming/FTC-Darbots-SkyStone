package org.darbots.darbotsftclib.libcore.odometry;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.darbots.darbotsftclib.libcore.calculations.dimentionalcalculation.Robot2DPositionIndicator;
import org.darbots.darbotsftclib.libcore.sensors.gyros.SynchronizedSoftwareGyro;
import org.darbots.darbotsftclib.libcore.templates.odometry.RobotPassive2DPositionTracker;
import org.darbots.darbotsftclib.libcore.templates.other_sensors.RobotGyro;

public class Robot2DPassive2WheelTracker extends RobotPassive2DPositionTracker {
    private class Robot2DPassive2WheelTracker_Runnable implements Runnable{
        private volatile int m_SleepTimeInMillis = 75;
        private volatile boolean m_IsRunning = false;

        private DcMotor m_DriveEncoder;
        private DcMotor m_StrafeEncoder;

        private RobotGyro m_Gyro = null;

        private volatile boolean m_DriveEncoderReversed = false;
        private volatile boolean m_StrafeEncoderReversed = false;

        private volatile double m_DriveEncoderCountsPerCM = 0;
        private volatile double m_StrafeEncoderCountsPerCM = 0;

        private int m_LastDriveEncoderCount = 0;
        private int m_LastStrafeEncoderCount = 0;
        private float m_LastGyroReading = 0.0f;

        public boolean isRunning(){
            return this.m_IsRunning;
        }

        @Override
        public void run() {
            m_IsRunning = true;

            m_DriveEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m_StrafeEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            m_LastDriveEncoderCount = m_DriveEncoder.getCurrentPosition();
            m_LastStrafeEncoderCount = m_StrafeEncoder.getCurrentPosition();
            this.m_Gyro.updateStatus();
            m_LastGyroReading = this.m_Gyro.getHeading();

            while(m_IsRunning){
                try{
                   Thread.sleep(m_SleepTimeInMillis);
                }catch(InterruptedException e){
                    e.printStackTrace();
                }

                this.m_Gyro.updateStatus();

                int newMidCount = m_StrafeEncoder.getCurrentPosition();
                int newLeftCount = m_DriveEncoder.getCurrentPosition();
                float newGyroReading = m_Gyro.getHeading();

                int deltaMidCount = newMidCount - m_LastStrafeEncoderCount;
                int deltaLeftCount = newLeftCount - m_LastDriveEncoderCount;

                if(m_StrafeEncoderReversed)
                    deltaMidCount = -deltaMidCount;
                if(m_DriveEncoderReversed)
                    deltaLeftCount = -deltaLeftCount;

                double deltaStrafeCM = deltaMidCount / this.m_StrafeEncoderCountsPerCM;
                double deltaDriveCM = deltaLeftCount / this.m_DriveEncoderCountsPerCM;

                double deltaXMoved = deltaDriveCM;
                double deltaYMoved = deltaStrafeCM;
                double deltaAngMoved = m_Gyro.getHeading() - m_LastGyroReading;

                if(m_Gyro.getHeadingRotationPositiveOrientation() == RobotGyro.HeadingRotationPositiveOrientation.Clockwise){
                    deltaAngMoved = -deltaAngMoved;
                }

                Robot2DPassive2WheelTracker.this.drive_MoveThroughRobotAxisOffset(new Robot2DPositionIndicator(
                        deltaXMoved,
                        deltaYMoved,
                        deltaAngMoved
                ));


                m_LastStrafeEncoderCount = newMidCount;
                m_LastDriveEncoderCount = newLeftCount;

            }
            m_IsRunning = false;
        }

        public void stop(){
            this.m_IsRunning = false;
        }
    }

    private double m_DriveEncoderCountsPerRev = 0;
    private double m_StrafeEncoderCountsPerRev = 0;

    private double m_DriveEncoderWheelCircumference = 0;
    private double m_StrafeEncoderWheelCircumference = 0;

    private Robot2DPassive2WheelTracker_Runnable m_RunnableTracking = null;
    private Thread m_TrackingThread = null;
    private boolean m_TrackingThreadRunned = false;


    public Robot2DPassive2WheelTracker(Robot2DPositionIndicator initialPosition, RobotGyro Gyro, DcMotor DriveEncoder, DcMotor StrafeEncoder, double DriveEncoderCountsPerRev, double DriveEncoderWheelRadius, double StrafeEncoderCountsPerRev, double StrafeEncoderWheelRadius) {
        super(initialPosition);
        __setupRunnable();
        __setupParams(Gyro,DriveEncoder,StrafeEncoder,DriveEncoderCountsPerRev,DriveEncoderWheelRadius,StrafeEncoderCountsPerRev,StrafeEncoderWheelRadius);
    }

    public Robot2DPassive2WheelTracker(Robot2DPositionIndicator initialPosition, double RobotWidth, double RobotHeight,RobotGyro Gyro, DcMotor DriveEncoder, DcMotor StrafeEncoder, double DriveEncoderCountsPerRev, double DriveEncoderWheelRadius, double StrafeEncoderCountsPerRev, double StrafeEncoderWheelRadius){
        super(initialPosition, RobotWidth, RobotHeight);
        __setupRunnable();
        __setupParams(Gyro,DriveEncoder,StrafeEncoder,DriveEncoderCountsPerRev,DriveEncoderWheelRadius,StrafeEncoderCountsPerRev,StrafeEncoderWheelRadius);
    }

    public Robot2DPassive2WheelTracker(Robot2DPassive2WheelTracker oldTracker) {
        super(oldTracker);
        __setupRunnable();
        __setupParams(oldTracker.getGyro(),oldTracker.getDriveEncoder(),oldTracker.getStrafeEncoder(),oldTracker.getDriveEncoderCountsPerRev(),oldTracker.getDriveEncoderWheelRadius(),oldTracker.getStrafeEncoderCountsPerRev(),oldTracker.getStrafeEncoderWheelRadius());
    }

    private void __setupRunnable(){
        this.m_RunnableTracking = new Robot2DPassive2WheelTracker_Runnable();
        this.m_TrackingThread = new Thread(this.m_RunnableTracking);
        this.m_TrackingThreadRunned = false;
    }

    private void __setupParams(RobotGyro Gyro, DcMotor DriveEncoder, DcMotor StrafeEncoder, double DriveEncoderCountsPerRev, double DriveEncoderWheelRadius, double StrafeEncoderCountsPerRev, double StrafeEncoderWheelRadius){
        this.m_RunnableTracking.m_Gyro = Gyro;

        this.m_RunnableTracking.m_DriveEncoder = DriveEncoder;
        this.m_RunnableTracking.m_StrafeEncoder = StrafeEncoder;

        this.m_DriveEncoderCountsPerRev = DriveEncoderCountsPerRev;
        this.m_DriveEncoderWheelCircumference = DriveEncoderWheelRadius * (2 * Math.PI);
        this.__recalculateDriveCountsPerCM();

        this.m_StrafeEncoderCountsPerRev = StrafeEncoderCountsPerRev;
        this.m_StrafeEncoderWheelCircumference = StrafeEncoderWheelRadius * (2 * Math.PI);
        this.__recalculateStrafeCountsPerCM();

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

    public RobotGyro getGyro(){
        return this.m_RunnableTracking.m_Gyro;
    }

    public DcMotor getDriveEncoder(){
        return this.m_RunnableTracking.m_DriveEncoder;
    }

    public DcMotor getStrafeEncoder(){
        return this.m_RunnableTracking.m_StrafeEncoder;
    }

    public boolean isDriveEncoderReversed(){
        return this.m_RunnableTracking.m_DriveEncoderReversed;
    }

    public void setDriveEncoderReversed(boolean Reversed){
        this.m_RunnableTracking.m_DriveEncoderReversed = Reversed;
    }

    public boolean isStrafeEncoderReversed(){
        return this.m_RunnableTracking.m_StrafeEncoderReversed;
    }

    public void setStrafeEncoderReversed(boolean Reversed){
        this.m_RunnableTracking.m_StrafeEncoderReversed = Reversed;
    }

    public double getDriveEncoderCountsPerRev(){
        return this.m_DriveEncoderCountsPerRev;
    }

    public void setDriveEncoderCountsPerRev(double CountsPerRev){
        this.m_DriveEncoderCountsPerRev = CountsPerRev;
        this.__recalculateDriveCountsPerCM();
    }

    public double getDriveEncoderWheelCircumference(){
        return this.m_DriveEncoderWheelCircumference;
    }

    public double getDriveEncoderWheelRadius(){
        return this.m_DriveEncoderWheelCircumference / (2 * Math.PI);
    }

    public void setDriveEncoderWheelCircumference(double Circumference){
        this.m_DriveEncoderWheelCircumference = Circumference;
        this.__recalculateDriveCountsPerCM();
    }

    public void setDriveEncoderWheelRadius(double Radius){
        this.setDriveEncoderWheelCircumference(Radius * (2 * Math.PI));
    }

    protected void __recalculateDriveCountsPerCM(){
        this.m_RunnableTracking.m_DriveEncoderCountsPerCM = this.m_DriveEncoderCountsPerRev * (1.0 / this.m_DriveEncoderWheelCircumference);
    }

    public double getStrafeEncoderCountsPerRev(){
        return this.m_StrafeEncoderCountsPerRev;
    }

    public void setStrafeEncoderCountsPerRev(double CountsPerRev){
        this.m_StrafeEncoderCountsPerRev = CountsPerRev;
        this.__recalculateStrafeCountsPerCM();
    }

    public double getStrafeEncoderWheelCircumference(){
        return this.m_StrafeEncoderWheelCircumference;
    }

    public double getStrafeEncoderWheelRadius(){
        return this.m_StrafeEncoderWheelCircumference / (2 * Math.PI);
    }

    public void setStrafeEncoderWheelCircumference(double Circumference){
        this.m_StrafeEncoderWheelCircumference = Circumference;
        this.__recalculateStrafeCountsPerCM();
    }

    public void setStrafeEncoderWheelRadius(double Radius){
        this.setStrafeEncoderWheelCircumference(Radius * (2 * Math.PI));
    }

    protected void __recalculateStrafeCountsPerCM(){
        this.m_RunnableTracking.m_StrafeEncoderCountsPerCM = this.m_StrafeEncoderCountsPerRev * (1.0 / this.m_StrafeEncoderWheelCircumference);
    }

    protected void drive_MoveThroughRobotAxisOffset(Robot2DPositionIndicator robotAxisValues) {
        Robot2DPositionIndicator tempField = this.fieldAxisFromRobotAxis(robotAxisValues);
        this.setCurrentPosition(tempField);
        this.offsetRelative(robotAxisValues);
    }
}