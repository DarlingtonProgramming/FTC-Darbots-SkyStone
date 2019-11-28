package org.darbots.darbotsftclib.libcore.odometry;

import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPose2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.XYPlaneCalculations;
import org.darbots.darbotsftclib.libcore.sensors.motion_related.RobotWheel;
import org.darbots.darbotsftclib.libcore.templates.motor_related.MotorType;
import org.darbots.darbotsftclib.libcore.templates.odometry.RobotActive2DPositionTracker;


public abstract class Robot2Wheel2DTracker extends RobotActive2DPositionTracker {
    private int m_LastDriveEncoderCount = 0;
    private int m_LastStrafeEncoderCount = 0;
    private float m_LastGyroReading = 0.0f;
    private double c_DRIVEENCODER_COUNTS_PER_CM, c_STRAFEENCODER_COUNTS_PER_CM;
    private double c_DRIVEENCODER_CM_PER_DEG, c_STRAFEENCODER_CM_PER_DEG;


    private boolean m_DriveEncoderReversed = false;
    private boolean m_StrafeEncoderReversed = false;

    private MotorType m_DriveEncoderMotorType, m_StrafeEncoderMotorType;
    private RobotWheel m_DriveEncoderWheel, m_StrafeEncoderWheel;

    public Robot2Wheel2DTracker(RobotPose2D initialPosition, boolean DriveEncoderReversed, MotorType DriveEncoderMotorType, RobotWheel DriveEncoderWheel, boolean StrafeEncoderReversed, MotorType StrafeEncoderMotorType, RobotWheel StrafeEncoderWheel) {
        super(initialPosition);
        this.m_DriveEncoderReversed = DriveEncoderReversed;
        this.m_DriveEncoderMotorType = DriveEncoderMotorType;
        this.m_DriveEncoderWheel = DriveEncoderWheel;
        this.m_StrafeEncoderReversed = StrafeEncoderReversed;
        this.m_StrafeEncoderMotorType = StrafeEncoderMotorType;
        this.m_StrafeEncoderWheel = StrafeEncoderWheel;
    }

    public Robot2Wheel2DTracker(Robot2Wheel2DTracker oldTracker) {
        super(oldTracker);
        this.m_DriveEncoderReversed = oldTracker.m_DriveEncoderReversed;
        this.m_DriveEncoderMotorType = oldTracker.m_DriveEncoderMotorType;
        this.m_DriveEncoderWheel = oldTracker.m_DriveEncoderWheel;
        this.m_StrafeEncoderReversed = oldTracker.m_StrafeEncoderReversed;
        this.m_StrafeEncoderMotorType = oldTracker.m_StrafeEncoderMotorType;
        this.m_StrafeEncoderWheel = oldTracker.m_StrafeEncoderWheel;
    }


    protected abstract void updateData();
    protected abstract int getDriveEncoderCount();
    protected abstract int getStrafeEncoderCount();
    protected abstract float getGyroReading();
    protected abstract boolean isGyroCounterClockwisePositive();

    @Override
    protected void __trackStart() {
        c_DRIVEENCODER_COUNTS_PER_CM = m_DriveEncoderMotorType.getCountsPerRev() / m_DriveEncoderWheel.getCircumference();
        c_STRAFEENCODER_COUNTS_PER_CM = m_StrafeEncoderMotorType.getCountsPerRev() / m_StrafeEncoderWheel.getCircumference();
        c_DRIVEENCODER_CM_PER_DEG = m_DriveEncoderWheel.getDistanceFromCenterOfRobot() * 2.0 * Math.PI / 360.0;
        c_STRAFEENCODER_CM_PER_DEG = m_StrafeEncoderWheel.getDistanceFromCenterOfRobot() * 2.0 * Math.PI / 360.0;

        this.updateData();

        m_LastDriveEncoderCount = this.getDriveEncoderCount();
        m_LastStrafeEncoderCount = this.getStrafeEncoderCount();

        m_LastGyroReading = this.getGyroReading();
    }

    @Override
    protected void __trackLoop(double secondsSinceLastLoop) {
        this.updateData();
        int newMidCount = this.getDriveEncoderCount();
        int newLeftCount = this.getStrafeEncoderCount();
        float newGyroReading = this.getGyroReading();

        int deltaMidCount = newMidCount - m_LastStrafeEncoderCount;
        int deltaLeftCount = newLeftCount - m_LastDriveEncoderCount;

        if(m_StrafeEncoderReversed)
            deltaMidCount = -deltaMidCount;
        if(m_DriveEncoderReversed)
            deltaLeftCount = -deltaLeftCount;

        double deltaStrafeCM = deltaMidCount / c_STRAFEENCODER_COUNTS_PER_CM;
        double deltaDriveCM = deltaLeftCount / c_DRIVEENCODER_COUNTS_PER_CM;
        double deltaAngMoved = newGyroReading - m_LastGyroReading;
        if(!this.isGyroCounterClockwisePositive()){
            deltaAngMoved = -deltaAngMoved;
        }


        double deltaXMoved = deltaDriveCM - deltaAngMoved * c_DRIVEENCODER_CM_PER_DEG;
        double deltaYMoved = deltaStrafeCM - deltaAngMoved * c_STRAFEENCODER_CM_PER_DEG;

        deltaAngMoved = XYPlaneCalculations.normalizeDeg(deltaAngMoved);

        deltaXMoved /= Robot2Wheel2DTracker.this.m_XDistanceFactor;
        deltaYMoved /= Robot2Wheel2DTracker.this.m_YDistanceFactor;

        RobotPose2D currentVelocityVector = new RobotPose2D(
                deltaXMoved / secondsSinceLastLoop,
                deltaYMoved / secondsSinceLastLoop,
                deltaAngMoved / secondsSinceLastLoop
        );


        Robot2Wheel2DTracker.this.drive_MoveThroughRobotAxisOffset(new RobotPose2D(
                deltaXMoved,
                deltaYMoved,
                deltaAngMoved
        ));

        Robot2Wheel2DTracker.this.setCurrentVelocityVector(currentVelocityVector);


        m_LastStrafeEncoderCount = newMidCount;
        m_LastDriveEncoderCount = newLeftCount;
        m_LastGyroReading = newGyroReading;
    }
}
