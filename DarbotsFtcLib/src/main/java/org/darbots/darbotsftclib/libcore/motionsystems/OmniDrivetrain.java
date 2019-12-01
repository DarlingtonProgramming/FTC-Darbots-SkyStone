package org.darbots.darbotsftclib.libcore.motionsystems;

import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPose2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.XYPlaneCalculations;
import org.darbots.darbotsftclib.libcore.sensors.motion_related.RobotMotion;
import org.darbots.darbotsftclib.libcore.templates.chassis_related.RobotMotionSystem;
import org.darbots.darbotsftclib.libcore.templates.motor_related.RobotMotor;
import org.darbots.darbotsftclib.libcore.templates.odometry.Robot2DPositionTracker;

/**
 * A class for controlling Omni Wheel Drivetrains.
 * Orders for Motors
 * i = 0, LT
 * i = 1, RT
 * i = 2, LB
 * i = 3, RB
 */
public class OmniDrivetrain extends RobotMotionSystem {
    private double c_Kwl, c_Kwr, c_Krl, c_Krr, c_Kd, c_Wmax, c_R;
    private RobotMotion m_LT, m_RT, m_LB, m_RB;
    RobotMotor m_LTMotor, m_RTMotor, m_LBMotor, m_RBMotor;
    public OmniDrivetrain(Robot2DPositionTracker PositionTracker, RobotMotion LT, RobotMotion RT, RobotMotion LB, RobotMotion RB) {
        super(PositionTracker);
        this.m_LT = LT;
        this.m_RT = RT;
        this.m_LB = LB;
        this.m_RB = RB;
        this.m_LTMotor = LT.getMotor();
        this.m_RTMotor = RT.getMotor();
        this.m_LBMotor = LB.getMotor();
        this.m_RBMotor = RB.getMotor();
    }

    protected void __calculateConstants(){
        double Lx = Math.abs(this.m_LT.getRobotWheel().getOnRobotPosition().X), Ly = Math.abs(this.m_LT.getRobotWheel().getOnRobotPosition().Y);
        this.c_Kd = this.m_LT.getRobotWheel().getOnRobotPosition().getDistanceToOrigin();
        this.c_Wmax = this.m_LTMotor.getMotorType().getRevPerSec() * 360.0;
        this.c_R = this.m_LT.getRobotWheel().getRadius();
        this.c_Kwl = XYPlaneCalculations.CONST_180_OVER_PI / (this.c_R * Math.sqrt(2));
        this.c_Kwr = this.c_Kd / this.c_R;
        this.c_Krl = XYPlaneCalculations.CONST_PI_OVER_180 * (Math.sqrt(2) * this.c_R / 4.0);
        this.c_Krr = this.c_R / (4.0 * this.c_Kd);
    }

    public OmniDrivetrain(RobotMotionSystem MotionSystem) {
        super(MotionSystem);
    }

    @Override
    protected void __stopMotion() {
        this.m_LTMotor.setPower(0);
        this.m_RTMotor.setPower(0);
        this.m_LBMotor.setPower(0);
        this.m_RBMotor.setPower(0);
    }

    @Override
    protected void __updateMotorStatus() {
        this.m_LTMotor.updateStatus();
        this.m_RTMotor.updateStatus();
        this.m_LBMotor.updateStatus();
        this.m_RBMotor.updateStatus();
    }

    @Override
    public RobotPose2D getTheoreticalMaximumMotionState(double WantedXSpeedInCMPerSec, double WantedYSpeedInCMPerSec, double WantedZRotSpeedInDegPerSec) {
        double scenarioW = this.c_Kwl * Math.abs(WantedXSpeedInCMPerSec) + this.c_Kwl * Math.abs(WantedYSpeedInCMPerSec) + this.c_Kwr * Math.abs(WantedZRotSpeedInDegPerSec);
        double fraction = this.c_Wmax / scenarioW;
        return new RobotPose2D(
                WantedXSpeedInCMPerSec * fraction,
                WantedYSpeedInCMPerSec * fraction,
                WantedZRotSpeedInDegPerSec * fraction
        );
    }

    @Override
    protected void __setRobotSpeed(double XSpeedInCMPerSec, double YSpeedInCMPerSec, double ZRotSpeedInDegPerSec) {
        double[] calculatedAngularSpeeds = this.calculateWheelAngularSpeeds(XSpeedInCMPerSec,YSpeedInCMPerSec,ZRotSpeedInDegPerSec);
        double[] normalizedWheelSpeeds = new double[calculatedAngularSpeeds.length];
        for(int i = 0; i < normalizedWheelSpeeds.length; i++){
            normalizedWheelSpeeds[i] = calculatedAngularSpeeds[i] / this.c_Wmax;
        }
        this.m_LTMotor.setPower(normalizedWheelSpeeds[0]);
        this.m_RTMotor.setPower(normalizedWheelSpeeds[1]);
        this.m_LBMotor.setPower(normalizedWheelSpeeds[2]);
        this.m_RBMotor.setPower(normalizedWheelSpeeds[3]);
    }

    @Override
    public double[] calculateWheelAngularSpeeds(double RobotXSpeedInCMPerSec, double RobotYSpeedInCMPerSec, double RobotZRotSpeedInDegPerSec) {
        double xSpeedVal = this.c_Kwl * RobotXSpeedInCMPerSec;
        double ySpeedVal = this.c_Kwl * RobotYSpeedInCMPerSec;
        double zRotSpeedVal = this.c_Kwr * RobotZRotSpeedInDegPerSec;
        double LTSpeed = - xSpeedVal + ySpeedVal + zRotSpeedVal;
        double RTSpeed = + xSpeedVal + ySpeedVal + zRotSpeedVal;
        double LBSpeed = - xSpeedVal - ySpeedVal + zRotSpeedVal;
        double RBSpeed = + xSpeedVal - ySpeedVal + zRotSpeedVal;
        double[] speedVals = {LTSpeed, RTSpeed, LBSpeed, RBSpeed};
        return speedVals;
    }

    @Override
    public RobotPose2D calculateRobotSpeed(double[] wheelSpeeds) {
        double LTSpeed = wheelSpeeds[0], RTSpeed = wheelSpeeds[1], LBSpeed = wheelSpeeds[2], RBSpeed = wheelSpeeds[3];
        double RobotXSpeed = this.c_Krl * (- LTSpeed + RTSpeed - LBSpeed + RBSpeed);
        double RobotYSpeed = this.c_Krl * (+ LTSpeed + RTSpeed - LBSpeed - RBSpeed);
        double RobotRotZSpeed = this.c_Krr * (+ LTSpeed + RTSpeed + LBSpeed + RBSpeed);
        return new RobotPose2D(RobotXSpeed, RobotYSpeed, RobotRotZSpeed);
    }

    @Override
    public double calculateMaxLinearSpeedCombinationsInCMPerSec(double angularSpeedInDegPerSec) {
        return (this.c_Wmax - this.c_Kwr * Math.abs(angularSpeedInDegPerSec)) / this.c_Krl;
    }

    @Override
    public double calculateMaxAngularSpeedInDegPerSec(double linearSpeedCombinationInCMPerSec) {
        return (this.c_Wmax - this.c_Kwl * Math.abs(linearSpeedCombinationInCMPerSec)) / this.c_Kwr;
    }

    public RobotMotion getLTMotion(){
        return this.m_LT;
    }

    public RobotMotion getRTMotion(){
        return this.m_RT;
    }

    public RobotMotion getLBMotion(){
        return this.m_LB;
    }

    public RobotMotion getRBMotion(){
        return this.m_RB;
    }
}
