package org.darbots.darbotsftclib.libcore.odometry;

import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPose2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotVector2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.XYPlaneCalculations;
import org.darbots.darbotsftclib.libcore.motionsystems.OmniDrivetrain;
import org.darbots.darbotsftclib.libcore.templates.motor_related.RobotMotor;
import org.darbots.darbotsftclib.libcore.templates.odometry.RobotActive2DPositionTracker;
import org.darbots.darbotsftclib.libcore.templates.other_sensors.RobotGyro;

public class OmniChassis2DPositionTracker extends RobotActive2DPositionTracker implements RobotGyro {
    private int m_LastLTEncoderCount, m_LastRTEncoderCount, m_LastLBEncoderCount, m_LastRBEncoderCount;
    private double c_LT_COUNTS_PER_DEG, c_RT_COUNTS_PER_DEG, c_LB_COUNTS_PER_DEG, c_RB_COUNTS_PER_DEG;
    private RobotMotor m_LTMotor, m_RTMotor, m_LBMotor, m_RBMotor;

    private OmniDrivetrain m_MotionSystem;
    private volatile double m_HeadingAngle = 0.0;

    public OmniChassis2DPositionTracker(RobotPose2D initialPosition, OmniDrivetrain driveTrain) {
        super(initialPosition);
        this.m_MotionSystem = driveTrain;
    }

    public OmniChassis2DPositionTracker(OmniChassis2DPositionTracker oldTracker) {
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

        double[] wheelSpeeds = {LTAngularSpeed, RTAngularSpeed, LBAngularSpeed, RBAngularSpeed};
        RobotVector2D chassisSpeed = m_MotionSystem.calculateRobotSpeed(wheelSpeeds);


        double deltaXMoved = chassisSpeed.X * secondsSinceLastLoop;
        double deltaYMoved = chassisSpeed.Y * secondsSinceLastLoop;
        double deltaAngMoved = chassisSpeed.getRotationZ() * secondsSinceLastLoop;


        __trackLoopMoved(
                chassisSpeed,
                new RobotPose2D(
                    deltaXMoved,
                    deltaYMoved,
                    deltaAngMoved
                )
        );

        offsetHeading(deltaAngMoved * OmniChassis2DPositionTracker.this.m_ZRotDistanceFactor);

        m_LastLTEncoderCount = newLTCount;
        m_LastRTEncoderCount = newRTCount;
        m_LastLBEncoderCount = newLBCount;
        m_LastRBEncoderCount = newRBCount;
    }

}
