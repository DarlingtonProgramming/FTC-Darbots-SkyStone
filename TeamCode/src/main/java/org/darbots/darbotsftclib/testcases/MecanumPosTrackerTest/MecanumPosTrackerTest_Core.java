package org.darbots.darbotsftclib.testcases.MecanumPosTrackerTest;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.darbots.darbotsftclib.libcore.calculations.dimentionalcalculation.Robot2DPositionIndicator;
import org.darbots.darbotsftclib.libcore.motortypes.AndyMark2964;
import org.darbots.darbotsftclib.libcore.motortypes.AndyMark3637;
import org.darbots.darbotsftclib.libcore.odometry.MecanumChassis2DPositionTracker;
import org.darbots.darbotsftclib.libcore.runtime.GlobalUtil;
import org.darbots.darbotsftclib.libcore.sensors.motion_related.RobotMotion;
import org.darbots.darbotsftclib.libcore.sensors.motion_related.RobotWheel;
import org.darbots.darbotsftclib.libcore.sensors.motors.RobotMotorController;
import org.darbots.darbotsftclib.libcore.sensors.motors.RobotMotorWithEncoder;
import org.darbots.darbotsftclib.libcore.templates.RobotCore;
import org.darbots.darbotsftclib.libcore.templates.chassis_related.RobotMotionSystem;
import org.darbots.darbotsftclib.libcore.templates.motor_related.MotorType;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MecanumPosTrackerTest_Core extends RobotCore {
    public static final double[] WheelPosition = {14,18.415};
    public static final MotorType ChassisMotorType = new AndyMark3637();
    public static final double WheelRadius = 5;

    private MecanumChassis2DPositionTracker m_PosTracker;

    public MecanumPosTrackerTest_Core(HardwareMap hardwareMap) {
        super("MecanumPosTrackerTest.log", hardwareMap);
        __initHardware(hardwareMap);
    }

    public MecanumPosTrackerTest_Core(HardwareMap hardwareMap, int ThreadPriority) {
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
        RobotWheel m_LeftTopWheel = new RobotWheel(new Robot2DPositionIndicator(WheelPosition[0],WheelPosition[1],45),WheelRadius);
        RobotWheel m_RightTopWheel = new RobotWheel(new Robot2DPositionIndicator(WheelPosition[0],-WheelPosition[1],-45),WheelRadius);
        RobotWheel m_LeftBottomWheel = new RobotWheel(new Robot2DPositionIndicator(WheelPosition[0],-WheelPosition[1],135),WheelRadius);
        RobotWheel m_RightBottomWheel = new RobotWheel(new Robot2DPositionIndicator(-WheelPosition[0],-WheelPosition[1],-135),WheelRadius);
        RobotMotion LTMotion = new RobotMotion(new RobotMotorController(new RobotMotorWithEncoder(m_LeftTopDC,ChassisMotorType),false,0),m_LeftTopWheel);
        RobotMotion LBMotion = new RobotMotion(new RobotMotorController(new RobotMotorWithEncoder(m_LeftBottomDC,ChassisMotorType),false,0),m_LeftBottomWheel);
        RobotMotion RTMotion = new RobotMotion(new RobotMotorController(new RobotMotorWithEncoder(m_RightTopDC,ChassisMotorType),false,0),m_RightTopWheel);
        RobotMotion RBMotion = new RobotMotion(new RobotMotorController(new RobotMotorWithEncoder(m_RightBottomDC,ChassisMotorType),false,0),m_RightBottomWheel);
        this.m_PosTracker = new MecanumChassis2DPositionTracker(new Robot2DPositionIndicator(0,0,0),false,LTMotion,RTMotion,LBMotion,RBMotion);
        this.m_PosTracker.start();
    }

    @Override
    public void stop() {

    }

    @Override
    public void terminate() {
        this.m_PosTracker.stop();
    }

    @Override
    public RobotMotionSystem getChassis() {
        return null;
    }

    @Override
    public void updateTelemetry() {
        Telemetry globalTele = GlobalUtil.getTelemetry();
        if(globalTele != null){
            Robot2DPositionIndicator currentPos = this.m_PosTracker.getCurrentPosition();
            globalTele.addLine("Current Position")
                    .addData("X", currentPos.getX())
                    .addData("Y",currentPos.getY())
                    .addData("RotZ", currentPos.getRotationZ());
        }
    }

    @Override
    public boolean isBusy() {
        return false;
    }

    @Override
    public void updateStatus() {

    }
}
