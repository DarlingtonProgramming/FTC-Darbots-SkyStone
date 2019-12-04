package org.darbots.darbotsftclib.testcases.MecanumMotionProfilingTest;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPose2D;
import org.darbots.darbotsftclib.libcore.motionsystems.MecanumDrivetrain;
import org.darbots.darbotsftclib.libcore.motortypes.AndyMark3637;
import org.darbots.darbotsftclib.libcore.odometry.MecanumChassis2DPositionTracker;
import org.darbots.darbotsftclib.libcore.runtime.GlobalUtil;
import org.darbots.darbotsftclib.libcore.sensors.motion_related.RobotMotion;
import org.darbots.darbotsftclib.libcore.sensors.motion_related.RobotWheel;
import org.darbots.darbotsftclib.libcore.sensors.motors.RobotMotorWithEncoder;
import org.darbots.darbotsftclib.libcore.templates.RobotCore;
import org.darbots.darbotsftclib.libcore.templates.chassis_related.RobotMotionSystem;
import org.darbots.darbotsftclib.libcore.templates.motor_related.MotorType;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MecanumMotionProfilingTest_Core extends RobotCore {
    public static final double[] WheelPosition = {14,18.415};
    public static final MotorType ChassisMotorType = new AndyMark3637();
    public static final double WheelRadius = 5;

    private MecanumDrivetrain m_Chassis;
    private MecanumChassis2DPositionTracker m_PosTracker;

    public MecanumMotionProfilingTest_Core(HardwareMap hardwareMap) {
        super("MecanumPosTrackerTest.log", hardwareMap);
        __initHardware(hardwareMap);
    }

    public MecanumMotionProfilingTest_Core(HardwareMap hardwareMap, int ThreadPriority) {
        super("MecanumPosTrackerTest.log", hardwareMap, ThreadPriority);
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
        this.m_PosTracker = new MecanumChassis2DPositionTracker(new RobotPose2D(0,0,0),this.m_Chassis);
        this.m_Chassis.setPositionTracker(this.m_PosTracker);
        this.m_PosTracker.start();
    }

    @Override
    protected void __stop() {
        this.m_Chassis.stop();
    }

    @Override
    protected void __terminate() {
        this.m_PosTracker.stop();
    }

    @Override
    public RobotMotionSystem getChassis() {
        return this.m_Chassis;
    }

    @Override
    public void updateTelemetry() {
        Telemetry globalTele = GlobalUtil.getTelemetry();
        if(globalTele != null){
            RobotPose2D currentPos = this.m_PosTracker.getCurrentPosition();
            RobotPose2D offsetPos = this.m_PosTracker.getRelativeOffset();
            globalTele.addLine("Current Position")
                    .addData("X", currentPos.X)
                    .addData("Y",currentPos.Y)
                    .addData("RotZ", currentPos.getRotationZ());
            globalTele.addLine("Current Offset")
                    .addData("X",offsetPos.X)
                    .addData("Y",offsetPos.Y)
                    .addData("RotZ",currentPos.getRotationZ());
        }
    }

    @Override
    public boolean isBusy() {
        return false;
    }

    @Override
    protected void __updateStatus() {
        this.m_Chassis.updateStatus();
    }
}