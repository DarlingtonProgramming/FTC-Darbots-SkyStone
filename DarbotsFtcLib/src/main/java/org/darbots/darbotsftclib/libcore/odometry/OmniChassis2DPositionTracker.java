package org.darbots.darbotsftclib.libcore.odometry;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.darbots.darbotsftclib.libcore.calculations.dimentionalcalculation.RobotPose2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentionalcalculation.XYPlaneCalculations;
import org.darbots.darbotsftclib.libcore.sensors.gyros.SynchronizedSoftwareGyro;
import org.darbots.darbotsftclib.libcore.sensors.motion_related.RobotMotion;
import org.darbots.darbotsftclib.libcore.sensors.motion_related.RobotWheel;
import org.darbots.darbotsftclib.libcore.templates.motor_related.RobotMotor;
import org.darbots.darbotsftclib.libcore.templates.odometry.RobotActive2DPositionTracker;
import org.darbots.darbotsftclib.libcore.templates.odometry.RobotSynchronized2DPositionTracker;

public class OmniChassis2DPositionTracker extends RobotActive2DPositionTracker {
    private class OmniChassis2DPositionTracker_Runnable implements Runnable{
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

        private double m_CONST_180_OVER_PIRTSQRT2 = 0;
        private double m_CONST_PIRTSQRT2_OVER_4T180 = 0;
        private double m_CONST_R_OVER_4TD = 0;
        private double m_CONST_D_OVER_R = 0;
        private double m_CONST_D = 0;
        private double m_CONST_POWER_PER_DEG_S = 0;



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
                    Thread.sleep(OmniChassis2DPositionTracker.this.m_ThreadSleepTimeInMs);
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

                double LTAngularSpeed = deltaLTDeg / secondsDriven;
                double RTAngularSpeed = deltaRTDeg / secondsDriven;
                double LBAngularSpeed = deltaLBDeg / secondsDriven;
                double RBAngularSpeed = deltaRBDeg / secondsDriven;

                /*
                Here we are not calculating the Speed of the Robot by using our static method
                This is because the calculation of some constants takes up a lot of time, and we don't want to waste our time on repeatedly calculating those constants.
                Those constants are calculated when we are initializing the MecanumChassis2DPositionTrackerClass, and they help us to reduce double multiplications in speed calculation.
                 */

                RobotPose2D chassisSpeed = new RobotPose2D(
                        (m_CONST_PIRTSQRT2_OVER_4T180 * (-LTAngularSpeed + RTAngularSpeed - LBAngularSpeed + RBAngularSpeed)) / OmniChassis2DPositionTracker.this.m_XDistanceFactor,
                        (m_CONST_PIRTSQRT2_OVER_4T180 * (LTAngularSpeed + RTAngularSpeed - LBAngularSpeed - RBAngularSpeed)) / OmniChassis2DPositionTracker.this.m_YDistanceFactor,
                (m_CONST_R_OVER_4TD * (LTAngularSpeed + RTAngularSpeed + LBAngularSpeed + RBAngularSpeed)) / OmniChassis2DPositionTracker.this.m_ZRotDistanceFactor
                );


                double deltaXMoved = chassisSpeed.getX() * secondsDriven;
                double deltaYMoved = chassisSpeed.getY() * secondsDriven;
                double deltaAngMoved = chassisSpeed.getRotationZ() * secondsDriven;


                OmniChassis2DPositionTracker.this.drive_MoveThroughRobotAxisOffset(new RobotPose2D(
                        deltaXMoved,
                        deltaYMoved,
                        deltaAngMoved
                ));

                OmniChassis2DPositionTracker.this.setCurrentVelocityVector(chassisSpeed);

                if(this.m_Gyro != null){
                    this.m_Gyro.offsetHeading((float) deltaAngMoved);
                }

                m_LastLTEncoderCount = newLTCount;
                m_LastRTEncoderCount = newRTCount;
                m_LastLBEncoderCount = newLBCount;
                m_LastRBEncoderCount = newRBCount;

                m_Time.reset();
            }
            m_IsRunning = false;
        }

        public void stop(){
            this.m_IsRunning = false;
        }
    }

    private OmniChassis2DPositionTracker_Runnable m_RunnableTracking = null;
    private Thread m_TrackingThread = null;
    private boolean m_TrackingThreadRunned = false;


    public OmniChassis2DPositionTracker(RobotPose2D initialPosition, boolean initSoftwareGyro, RobotMotion LTMotion, RobotMotion RTMotion, RobotMotion LBMotion, RobotMotion RBMotion){
        super(initialPosition);
        __setupRunnable();
        __setupParams(initSoftwareGyro, LTMotion, RTMotion, LBMotion, RBMotion);
    }

    public OmniChassis2DPositionTracker(OmniChassis2DPositionTracker oldTracker) {
        super(oldTracker);
        __setupRunnable();
        __setupParams(oldTracker.getGyro() != null, oldTracker.getLTMotion(), oldTracker.getRTMotion(), oldTracker.getLBMotion(), oldTracker.getRBMotion());
    }

    private void __setupRunnable(){
        this.m_RunnableTracking = new OmniChassis2DPositionTracker_Runnable();
        this.m_TrackingThread = new Thread(this.m_RunnableTracking);
        this.m_TrackingThreadRunned = false;
    }

    private void __setupParams(boolean initSoftwareGyro, RobotMotion LTMotion, RobotMotion RTMotion, RobotMotion LBMotion, RobotMotion RBMotion) {
        this.m_RunnableTracking.LTMotion = LTMotion;
        this.m_RunnableTracking.RTMotion = RTMotion;
        this.m_RunnableTracking.LBMotion = LBMotion;
        this.m_RunnableTracking.RBMotion = RBMotion;

        double CONST_SQRT2 = Math.sqrt(2);

        this.m_RunnableTracking.m_CONST_D = Math.sqrt(Math.pow(LTMotion.getRobotWheel().getOnRobotPosition().getX(),2) + Math.pow(LTMotion.getRobotWheel().getOnRobotPosition().getY(),2));

        this.m_RunnableTracking.m_CONST_180_OVER_PIRTSQRT2 = XYPlaneCalculations.CONST_180_OVER_PI / (LTMotion.getRobotWheel().getRadius() * CONST_SQRT2);
        this.m_RunnableTracking.m_CONST_PIRTSQRT2_OVER_4T180 = XYPlaneCalculations.CONST_PI_OVER_180 * (LTMotion.getRobotWheel().getRadius() * CONST_SQRT2) / 4.0;
        this.m_RunnableTracking.m_CONST_R_OVER_4TD = LTMotion.getRobotWheel().getRadius() / (4 * this.m_RunnableTracking.m_CONST_D);
        this.m_RunnableTracking.m_CONST_D_OVER_R = (this.m_RunnableTracking.m_CONST_D / LTMotion.getRobotWheel().getRadius());

        this.m_RunnableTracking.m_CONST_LTCountsPerDeg = LTMotion.getMotorController().getMotor().getMotorType().getCountsPerRev() / 360.0;
        this.m_RunnableTracking.m_CONST_RTCountsPerDeg = RTMotion.getMotorController().getMotor().getMotorType().getCountsPerRev() / 360.0;
        this.m_RunnableTracking.m_CONST_LBCountsPerDeg = LBMotion.getMotorController().getMotor().getMotorType().getCountsPerRev() / 360.0;
        this.m_RunnableTracking.m_CONST_RBCountsPerDeg = RBMotion.getMotorController().getMotor().getMotorType().getCountsPerRev() / 360.0;
        this.m_RunnableTracking.m_CONST_POWER_PER_DEG_S = 1.0 / LTMotion.getMotorController().getMotor().getMotorType().getRevPerSec() / 360.0;

        if(initSoftwareGyro){
            this.m_RunnableTracking.m_Gyro = new SynchronizedSoftwareGyro(0.0f);
        }
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

    protected void drive_MoveThroughRobotAxisOffset(RobotPose2D robotAxisValues) {
        RobotPose2D tempField = this.fieldAxisFromRobotAxis(robotAxisValues);
        this.setCurrentPosition(tempField);
        this.offsetRelative(robotAxisValues);
    }

    public RobotPose2D calculateRobotSpeed(double LTAngularSpeed, double RTAngularSpeed, double LBAngularSpeed, double RBAngularSpeed){
        double xSpeed = this.m_RunnableTracking.m_CONST_PIRTSQRT2_OVER_4T180 * (-LTAngularSpeed + RTAngularSpeed - LBAngularSpeed + RBAngularSpeed);
        double ySpeed = this.m_RunnableTracking.m_CONST_PIRTSQRT2_OVER_4T180 * (LTAngularSpeed + RTAngularSpeed - LBAngularSpeed - RBAngularSpeed);
        double angularSpeed = this.m_RunnableTracking.m_CONST_R_OVER_4TD * (LTAngularSpeed + RTAngularSpeed + LBAngularSpeed + RBAngularSpeed);
        return new RobotPose2D(xSpeed,ySpeed,angularSpeed);
    }

    public double[] calculateWheelAngularSpeedInDegPerSec(RobotPose2D RobotSpeed){
        double xSpeed = RobotSpeed.getX();
        double ySpeed = RobotSpeed.getY();
        double rotSpeed = RobotSpeed.getRotationZ();

        double xSpeedConst = this.m_RunnableTracking.m_CONST_180_OVER_PIRTSQRT2 * xSpeed;
        double ySpeedConst = this.m_RunnableTracking.m_CONST_180_OVER_PIRTSQRT2 * ySpeed;
        double rotSpeedConst = this.m_RunnableTracking.m_CONST_D_OVER_R * rotSpeed;

        double LTSpeed = - xSpeedConst + ySpeedConst + rotSpeedConst;
        double RTSpeed = xSpeedConst + ySpeedConst + rotSpeedConst;
        double LBSpeed = -xSpeedConst - ySpeedConst + rotSpeedConst;
        double RBSpeed = xSpeedConst - ySpeedConst + rotSpeedConst;
        double[] speedReturn = {LTSpeed, RTSpeed, LBSpeed, RBSpeed};
        return speedReturn;
    }

    public double[] calculateMotorPowers(RobotPose2D RobotSpeed){
        double[] WheelDegPerSec = calculateWheelAngularSpeedInDegPerSec(RobotSpeed);
        double[] MotorSpeed = new double[WheelDegPerSec.length];
        for(int i = 0; i < WheelDegPerSec.length; i++) {
            MotorSpeed[i] = this.m_RunnableTracking.m_CONST_POWER_PER_DEG_S * WheelDegPerSec[0];
        }
        return MotorSpeed;
    }


    public static RobotPose2D calculateRobotSpeed(double LTAngularSpeed, double RTAngularSpeed, double LBAngularSpeed, double RBAngularSpeed, RobotWheel LTWheel){
        double CONST_D = Math.sqrt(Math.pow(LTWheel.getOnRobotPosition().getX(),2) + Math.pow(LTWheel.getOnRobotPosition().getY(),2));
        double CONST_SQRT2 = Math.sqrt(2);
        double CONST_PIRTSQRT2_OVER_4T180 = XYPlaneCalculations.CONST_PI_OVER_180 * LTWheel.getRadius() * CONST_SQRT2 / 4.0;
        double CONST_R_OVER_4TD = LTWheel.getRadius() / (4 * CONST_D);
        double xSpeed = CONST_PIRTSQRT2_OVER_4T180 * (-LTAngularSpeed + RTAngularSpeed - LBAngularSpeed + RBAngularSpeed);
        double ySpeed = CONST_PIRTSQRT2_OVER_4T180 * (LTAngularSpeed + RTAngularSpeed - LBAngularSpeed - RBAngularSpeed);
        double angularSpeed = CONST_R_OVER_4TD * (LTAngularSpeed + RTAngularSpeed + LBAngularSpeed + RBAngularSpeed);
        return new RobotPose2D(xSpeed,ySpeed,angularSpeed);
    }

    /**
     * This method calculates the speed of each wheel on the Mecanum chassis, given the desired velocity vector of the robot and the configuration of the robot.
     * @param RobotSpeed The velocity vector of the robot, in CM/s and deg/s
     * @param LTWheel The LTWheel RobotMotion class of the
     * @return The angular speed of each motor, in deg per sec, in order of LT, RT, LB, RB.
     */
    public static double[] calculateWheelAngularSpeedInDegPerSec(RobotPose2D RobotSpeed, RobotWheel LTWheel){
        double xSpeed = RobotSpeed.getX();
        double ySpeed = RobotSpeed.getY();
        double rotSpeed = RobotSpeed.getRotationZ();

        double CONST_D = Math.sqrt(Math.pow(LTWheel.getOnRobotPosition().getX(),2) + Math.pow(LTWheel.getOnRobotPosition().getY(),2));
        double CONST_SQRT2 = Math.sqrt(2);
        double CONST_180_OVER_PIRTSQRT2 = XYPlaneCalculations.CONST_180_OVER_PI / (LTWheel.getRadius() * CONST_SQRT2);
        double CONST_D_OVER_R = CONST_D / LTWheel.getRadius();

        double xSpeedConst = CONST_180_OVER_PIRTSQRT2 * xSpeed;
        double ySpeedConst = CONST_180_OVER_PIRTSQRT2 * ySpeed;
        double rotSpeedConst = CONST_D_OVER_R * rotSpeed;

        double LTSpeed = - xSpeedConst + ySpeedConst + rotSpeedConst;
        double RTSpeed = xSpeedConst + ySpeedConst + rotSpeedConst;
        double LBSpeed = -xSpeedConst - ySpeedConst + rotSpeedConst;
        double RBSpeed = xSpeedConst - ySpeedConst + rotSpeedConst;
        double[] speedReturn = {LTSpeed, RTSpeed, LBSpeed, RBSpeed};
        return speedReturn;
    }

}
