package org.darbots.darbotsftclib.libcore.odometry;

import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPose2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotVector2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.XYPlaneCalculations;
import org.darbots.darbotsftclib.libcore.motionsystems.MecanumDrivetrain;
import org.darbots.darbotsftclib.libcore.templates.motor_related.RobotMotor;
import org.darbots.darbotsftclib.libcore.templates.odometry.RobotActive2DPositionTracker;
import org.darbots.darbotsftclib.libcore.templates.other_sensors.RobotGyro;

public class MecanumChassis2DPositionTracker extends RobotActive2DPositionTracker implements RobotGyro {
    private int m_LastLTEncoderCount, m_LastRTEncoderCount, m_LastLBEncoderCount, m_LastRBEncoderCount;
    private double c_LT_COUNTS_PER_DEG, c_RT_COUNTS_PER_DEG, c_LB_COUNTS_PER_DEG, c_RB_COUNTS_PER_DEG;
    private RobotMotor m_LTMotor, m_RTMotor, m_LBMotor, m_RBMotor;

    private MecanumDrivetrain m_MotionSystem;
    private volatile double m_HeadingAngle = 0.0;

    public MecanumChassis2DPositionTracker(RobotPose2D initialPosition, MecanumDrivetrain driveTrain) {
        super(initialPosition);
        this.m_MotionSystem = driveTrain;
    }

    public MecanumChassis2DPositionTracker(MecanumChassis2DPositionTracker oldTracker) {
        super(oldTracker);
        this.m_MotionSystem = oldTracker.m_MotionSystem;
        this.m_HeadingAngle = oldTracker.m_HeadingAngle;
    }

    @Override
    public float getHeading() {
        return (float) this.m_HeadingAngle;
    }

    @Override
    public HeadingRotationPositiveOrientation getHeadingRotationPositiveOrientation() {
        return HeadingRotationPositiveOrientation.CounterClockwise;
    }

    private void offsetHeading(double offsetAngle){
        this.m_HeadingAngle = XYPlaneCalculations.normalizeDeg(this.m_HeadingAngle + offsetAngle);
        return;
    }

    @Override
    protected void __trackStart() {
        m_LTMotor = m_MotionSystem.getLTMotion().getMotor();
        m_RTMotor = m_MotionSystem.getRTMotion().getMotor();
        m_LBMotor = m_MotionSystem.getLBMotion().getMotor();
        m_RBMotor = m_MotionSystem.getRBMotion().getMotor();
        c_LT_COUNTS_PER_DEG = m_LTMotor.getMotorType().getCountsPerRev() / 360.0;
        c_RT_COUNTS_PER_DEG = m_RTMotor.getMotorType().getCountsPerRev() / 360.0;
        c_LB_COUNTS_PER_DEG = m_LBMotor.getMotorType().getCountsPerRev() / 360.0;
        c_RB_COUNTS_PER_DEG = m_RBMotor.getMotorType().getCountsPerRev() / 360.0;

        m_LastLTEncoderCount = m_LTMotor.getCurrentCount();
        m_LastRTEncoderCount = m_RTMotor.getCurrentCount();
        m_LastLBEncoderCount = m_LBMotor.getCurrentCount();
        m_LastRBEncoderCount = m_RBMotor.getCurrentCount();
    }

    @Override
    protected void __trackLoop(double secondsSinceLastLoop) {
        int newLTCount = m_LTMotor.getCurrentCount();
        int newRTCount = m_RTMotor.getCurrentCount();
        int newLBCount = m_LBMotor.getCurrentCount();
        int newRBCount = m_RBMotor.getCurrentCount();

        int deltaLTCount = newLTCount - m_LastLTEncoderCount;
        int deltaRTCount = newRTCount - m_LastRTEncoderCount;
        int deltaLBCount = newLBCount - m_LastLBEncoderCount;
        int deltaRBCount = newRBCount - m_LastRBEncoderCount;

        double deltaLTDeg = deltaLTCount / c_LT_COUNTS_PER_DEG;
        double deltaRTDeg = deltaRTCount / c_RT_COUNTS_PER_DEG;
        double deltaLBDeg = deltaLBCount / c_LB_COUNTS_PER_DEG;
        double deltaRBDeg = deltaRBCount / c_RB_COUNTS_PER_DEG;

        double LTAngularSpeed = deltaLTDeg / secondsSinceLastLoop;
        double RTAngularSpeed = deltaRTDeg / secondsSinceLastLoop;
        double LBAngularSpeed = deltaLBDeg / secondsSinceLastLoop;
        double RBAngularSpeed = deltaRBDeg / secondsSinceLastLoop;

                /*
                Here we are not calculating the Speed of the Robot by using our static method
                This is because the calculation of some constants takes up a lot of time, and we don't want to waste our time on repeatedly calculating those constants.
                Those constants are calculated when we are initializing the MecanumChassis2DPositionTrackerClass, and they help us to reduce double multiplications in speed calculation.
                 */

        double[] wheelSpeeds = {LTAngularSpeed, RTAngularSpeed, LBAngularSpeed, RBAngularSpeed};
        RobotVector2D chassisSpeed = m_MotionSystem.calculateRobotSpeed(wheelSpeeds);


        double deltaXMoved = chassisSpeed.X * secondsSinceLastLoop;
        double deltaYMoved = chassisSpeed.Y * secondsSinceLastLoop;
        double deltaAngMoved = chassisSpeed.getRotationZ() * secondsSinceLastLoop;


        MecanumChassis2DPositionTracker.this.drive_MoveThroughRobotAxisOffset(new RobotPose2D(
                deltaXMoved,
                deltaYMoved,
                deltaAngMoved
        ));

        MecanumChassis2DPositionTracker.this.setCurrentVelocityVector(chassisSpeed);

        offsetHeading(deltaAngMoved);

        m_LastLTEncoderCount = newLTCount;
        m_LastRTEncoderCount = newRTCount;
        m_LastLBEncoderCount = newLBCount;
        m_LastRBEncoderCount = newRBCount;
    }

}
