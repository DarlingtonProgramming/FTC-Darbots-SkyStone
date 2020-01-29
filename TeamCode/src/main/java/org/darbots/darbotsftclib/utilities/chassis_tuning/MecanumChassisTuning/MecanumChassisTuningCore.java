package org.darbots.darbotsftclib.utilities.chassis_tuning.MecanumChassisTuning;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPose2D;
import org.darbots.darbotsftclib.libcore.motionsystems.MecanumDrivetrain;
import org.darbots.darbotsftclib.libcore.motortypes.AndyMark3637;
import org.darbots.darbotsftclib.libcore.odometry.MecanumOdometry;
import org.darbots.darbotsftclib.libcore.sensors.motion_related.RobotMotion;
import org.darbots.darbotsftclib.libcore.sensors.motion_related.RobotWheel;
import org.darbots.darbotsftclib.libcore.sensors.motors.RobotMotorWithEncoder;
import org.darbots.darbotsftclib.libcore.templates.RobotCore;
import org.darbots.darbotsftclib.libcore.templates.chassis_related.RobotMotionSystem;
import org.darbots.darbotsftclib.libcore.templates.motor_related.MotorType;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MecanumChassisTuningCore extends RobotCore {
    public static final double[] WheelPosition = {28.1/2,36.5/2}; //{RobotLength / 2, RobotWidth / 2
    public static final MotorType ChassisMotorType = new AndyMark3637();
    public static final double WheelRadius = 5;

    private MecanumDrivetrain m_Chassis;

    public MecanumChassisTuningCore(String logFileName, HardwareMap hardwareMap) {
        super(logFileName, hardwareMap);
        __initHardware(hardwareMap);
    }

    public MecanumChassisTuningCore(String logFileName, HardwareMap hardwareMap, int ThreadPriority) {
        super(logFileName, hardwareMap, ThreadPriority);
        __initHardware(hardwareMap);
    }

    protected void __initHardware(HardwareMap hardwareMap){
        DcMotor m_LeftTopDC = hardwareMap.dcMotor.get("LF");
        DcMotor m_LeftBottomDC = hardwareMap.dcMotor.get("LB");
        DcMotor m_RightTopDC = hardwareMap.dcMotor.get("RF");
        DcMotor m_RightBottomDC = hardwareMap.dcMotor.get("RB");
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
        MecanumOdometry posTracker = new MecanumOdometry(new RobotPose2D(0,0,0),this.m_Chassis);
        this.m_Chassis.setPositionTracker(posTracker);
        posTracker.start();
    }

    @Override
    protected void __stop() {
        m_Chassis.stop();
    }

    @Override
    protected void __terminate() {
        m_Chassis.terminate();
    }

    @Override
    public RobotMotionSystem getChassis() {
        return m_Chassis;
    }

    @Override
    protected void __updateStatus() {
        m_Chassis.updateStatus();
    }

    @Override
    public void __updateTelemetry(Telemetry telemetry, TelemetryPacket telemetryPacket) {

    }

    @Override
    public boolean isBusy() {
        return false;
    }
}
