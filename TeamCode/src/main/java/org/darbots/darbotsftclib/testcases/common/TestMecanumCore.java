package org.darbots.darbotsftclib.testcases.common;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPose2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotVector2D;
import org.darbots.darbotsftclib.libcore.motionsystems.MecanumDrivetrain;
import org.darbots.darbotsftclib.libcore.motortypes.AndyMark3637;
import org.darbots.darbotsftclib.libcore.odometry.MecanumOdometry;
import org.darbots.darbotsftclib.libcore.sensors.motion_related.RobotMotion;
import org.darbots.darbotsftclib.libcore.sensors.motion_related.RobotWheel;
import org.darbots.darbotsftclib.libcore.sensors.motors.RobotMotorWithEncoder;
import org.darbots.darbotsftclib.libcore.templates.RobotCore;
import org.darbots.darbotsftclib.libcore.templates.RobotNonBlockingDevice;
import org.darbots.darbotsftclib.libcore.templates.chassis_related.RobotMotionSystem;
import org.darbots.darbotsftclib.libcore.templates.motor_related.MotorType;
import org.darbots.darbotsftclib.libcore.templates.odometry.RobotSeparateThreadPositionTracker;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TestMecanumCore extends RobotCore {
    public static final double[] WheelPosition = {28.1/2,36.5/2};
    public static final MotorType ChassisMotorType = new AndyMark3637();
    public static final double WheelRadius = 5;
    public static final double XFactor = 1.0, YFactor = 1.09, RotZFactor = 1.070395;

    private MecanumDrivetrain m_Chassis;
    private RobotSeparateThreadPositionTracker m_PosTracker;

    public TestMecanumCore(HardwareMap hardwareMap, String logFileName) {
        super(logFileName, hardwareMap);
        __initHardware(hardwareMap);
    }

    public TestMecanumCore(HardwareMap hardwareMap, String logFileName, int ThreadPriority) {
        super(logFileName, hardwareMap, ThreadPriority);
        __initHardware(hardwareMap);
    }

    protected void __initHardware(HardwareMap map){
        DcMotor m_LeftTopDC = map.dcMotor.get("LF");
        DcMotor m_LeftBottomDC = map.dcMotor.get("LB");
        DcMotor m_RightTopDC = map.dcMotor.get("RF");
        DcMotor m_RightBottomDC = map.dcMotor.get("RB");
        m_LeftTopDC.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        m_LeftBottomDC.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        m_RightTopDC.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        m_RightBottomDC.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        RobotWheel m_LeftTopWheel = new RobotWheel(new RobotPose2D(WheelPosition[0],WheelPosition[1],45),WheelRadius);
        RobotWheel m_RightTopWheel = new RobotWheel(new RobotPose2D(WheelPosition[0],-WheelPosition[1],-45),WheelRadius);
        RobotWheel m_LeftBottomWheel = new RobotWheel(new RobotPose2D(WheelPosition[0],-WheelPosition[1],135),WheelRadius);
        RobotWheel m_RightBottomWheel = new RobotWheel(new RobotPose2D(-WheelPosition[0],-WheelPosition[1],-135),WheelRadius);
        RobotMotion LTMotion = new RobotMotion(new RobotMotorWithEncoder(m_LeftTopDC,ChassisMotorType),m_LeftTopWheel);
        RobotMotion LBMotion = new RobotMotion(new RobotMotorWithEncoder(m_LeftBottomDC,ChassisMotorType),m_LeftBottomWheel);
        RobotMotion RTMotion = new RobotMotion(new RobotMotorWithEncoder(m_RightTopDC,ChassisMotorType),m_RightTopWheel);
        RobotMotion RBMotion = new RobotMotion(new RobotMotorWithEncoder(m_RightBottomDC,ChassisMotorType),m_RightBottomWheel);

        this.m_Chassis = new MecanumDrivetrain(null,LTMotion,RTMotion,LBMotion,RBMotion);
        this.m_Chassis.setLinearXMotionDistanceFactor(XFactor);
        this.m_Chassis.setLinearYMotionDistanceFactor(YFactor);
        this.m_Chassis.setRotationalMotionDistanceFactor(RotZFactor);
        this.m_PosTracker = new RobotSeparateThreadPositionTracker(new MecanumOdometry(this.m_Chassis), new RobotPose2D(0,0,0));
        this.m_PosTracker.setDistanceFactors(new RobotVector2D(XFactor,YFactor,RotZFactor));
        this.m_Chassis.setPositionTracker(this.m_PosTracker);
        this.m_PosTracker.start();
    }

    @Override
    protected void __stop() {
        this.m_Chassis.stop();
    }

    @Override
    protected void __terminate() {
        this.m_Chassis.terminate();
    }

    @Override
    public RobotMotionSystem getChassis() {
        return this.m_Chassis;
    }

    @Override
    public boolean isBusy() {
        return false;
    }

    @Override
    protected void __updateStatus() {
        this.m_Chassis.updateStatus();
        if(this.getGyro() != null && this.getGyro() instanceof RobotNonBlockingDevice){
            ((RobotNonBlockingDevice) this.getGyro()).updateStatus();
        }
    }

    @Override
    public void __updateTelemetry(Telemetry telemetry, TelemetryPacket telemetryPacket) {

    }
}
