package org.firstinspires.ftc.teamcode.david_cao.Gen4_SwanSilver_Code;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPose2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotVector2D;
import org.darbots.darbotsftclib.libcore.integratedfunctions.FTCMemory;
import org.darbots.darbotsftclib.libcore.motionsystems.MecanumDrivetrain;
import org.darbots.darbotsftclib.libcore.motortypes.AndyMark3637;
import org.darbots.darbotsftclib.libcore.motortypes.GoBilda5202Series1150RPMMotor;
import org.darbots.darbotsftclib.libcore.motortypes.MotorTypeUtil;
import org.darbots.darbotsftclib.libcore.odometry.MecanumChassis2DPositionTracker;
import org.darbots.darbotsftclib.libcore.runtime.GlobalUtil;
import org.darbots.darbotsftclib.libcore.runtime.SensorUtil;
import org.darbots.darbotsftclib.libcore.sensors.motion_related.RobotMotion;
import org.darbots.darbotsftclib.libcore.sensors.motion_related.RobotWheel;
import org.darbots.darbotsftclib.libcore.sensors.motors.RobotMotorController;
import org.darbots.darbotsftclib.libcore.sensors.motors.RobotMotorWithEncoder;
import org.darbots.darbotsftclib.libcore.sensors.servos.motor_powered_servos.RobotServoUsingMotor;
import org.darbots.darbotsftclib.libcore.templates.RobotCore;
import org.darbots.darbotsftclib.libcore.templates.RobotNonBlockingDevice;
import org.darbots.darbotsftclib.libcore.templates.chassis_related.RobotMotionSystem;
import org.darbots.darbotsftclib.libcore.templates.motor_related.MotorType;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SwanSilverCore extends RobotCore {

    private MecanumDrivetrain m_Chassis;
    private MecanumChassis2DPositionTracker m_PosTracker;
    public RobotServoUsingMotor Slide;
    private Servo m_Grabber;
    public Servo GrabberMover;
    private Servo m_FoundationGrabberL, m_FoundationGrabberR;

    public SwanSilverCore(HardwareMap hardwareMap, String logFileName) {
        super(logFileName, hardwareMap);
        __initHardware(hardwareMap);
    }

    public SwanSilverCore(HardwareMap hardwareMap, String logFileName, int ThreadPriority) {
        super(logFileName, hardwareMap, ThreadPriority);
        __initHardware(hardwareMap);
    }

    public void readPosition(){
        RobotPose2D readPosition = FTCMemory.getSetting("SwanSilverPosition",this.getChassis().getPositionTracker().getCurrentPosition());
        if(readPosition != null) {
            this.m_PosTracker.setCurrentPosition(readPosition);
        }
    }

    public void savePosition(){
        FTCMemory.setSetting("SwanSilverPosition",this.getChassis().getPositionTracker().getCurrentPosition());
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
        RobotWheel m_LeftTopWheel = new RobotWheel(new RobotPose2D(SwanSilverSettings.CHASSIS_LENGTH / 2,SwanSilverSettings.CHASSIS_WIDTH / 2,45),SwanSilverSettings.CHASSIS_WHEEL_RADIUS);
        RobotWheel m_RightTopWheel = new RobotWheel(new RobotPose2D(SwanSilverSettings.CHASSIS_LENGTH / 2,-SwanSilverSettings.CHASSIS_WIDTH / 2,-45),SwanSilverSettings.CHASSIS_WHEEL_RADIUS);
        RobotWheel m_LeftBottomWheel = new RobotWheel(new RobotPose2D(SwanSilverSettings.CHASSIS_LENGTH / 2,-SwanSilverSettings.CHASSIS_WIDTH / 2,135),SwanSilverSettings.CHASSIS_WHEEL_RADIUS);
        RobotWheel m_RightBottomWheel = new RobotWheel(new RobotPose2D(-SwanSilverSettings.CHASSIS_LENGTH / 2,-SwanSilverSettings.CHASSIS_WIDTH / 2,-135),SwanSilverSettings.CHASSIS_WHEEL_RADIUS);
        RobotMotion LTMotion = new RobotMotion(new RobotMotorWithEncoder(m_LeftTopDC,SwanSilverSettings.CHASSIS_MOTOR_TYPE),m_LeftTopWheel);
        RobotMotion LBMotion = new RobotMotion(new RobotMotorWithEncoder(m_LeftBottomDC,SwanSilverSettings.CHASSIS_MOTOR_TYPE),m_LeftBottomWheel);
        RobotMotion RTMotion = new RobotMotion(new RobotMotorWithEncoder(m_RightTopDC,SwanSilverSettings.CHASSIS_MOTOR_TYPE),m_RightTopWheel);
        RobotMotion RBMotion = new RobotMotion(new RobotMotorWithEncoder(m_RightBottomDC,SwanSilverSettings.CHASSIS_MOTOR_TYPE),m_RightBottomWheel);

        this.m_Chassis = new MecanumDrivetrain(null,LTMotion,RTMotion,LBMotion,RBMotion);
        this.m_Chassis.setLinearXMotionDistanceFactor(SwanSilverSettings.CHASSIS_FACTORS.X);
        this.m_Chassis.setLinearYMotionDistanceFactor(SwanSilverSettings.CHASSIS_FACTORS.Y);
        this.m_Chassis.setRotationalMotionDistanceFactor(SwanSilverSettings.CHASSIS_FACTORS.getRotationZ());

        this.m_PosTracker = new MecanumChassis2DPositionTracker(new RobotPose2D(0,0,0),this.m_Chassis);
        this.m_PosTracker.setDistanceFactors(SwanSilverSettings.CHASSIS_FACTORS);

        this.m_Chassis.setPositionTracker(this.m_PosTracker);
        this.m_PosTracker.start();

        DcMotor linearSlideDCMotor = map.dcMotor.get("motorLinearSlide");
        RobotMotorWithEncoder linearSlideMotor = new RobotMotorWithEncoder(linearSlideDCMotor,SwanSilverSettings.LINEAR_SLIDE_MOTORTYPE);
        RobotMotorController linearSlideMotorController = new RobotMotorController(linearSlideMotor,SwanSilverSettings.LINEAR_SLIDE_TIMEOUT,SwanSilverSettings.LINEAR_SLIDE_TIMEOUT_FACTOR);

        this.Slide = new RobotServoUsingMotor(linearSlideMotorController,SwanSilverSettings.LINEAR_SLIDE_INIT,SwanSilverSettings.LINEAR_SLIDE_MIN,SwanSilverSettings.LINEAR_SLIDE_MAX);
        this.m_Grabber = map.servo.get("servoGrabber");
        SensorUtil.setServoPulseWidth(this.m_Grabber,SwanSilverSettings.GRABBER_SERVO_TYPE);
        this.GrabberMover = map.servo.get("servoGrabberMover");
        SensorUtil.setServoPulseWidth(this.GrabberMover,SwanSilverSettings.GRABBERMOVER_SERVO_TYPE);
        this.GrabberMover.scaleRange(SwanSilverSettings.GRABBERMOVER_MIN,SwanSilverSettings.GRABBERMOVER_MAX);
        this.GrabberMover.setPosition(0.5);

        this.m_FoundationGrabberL = map.servo.get("servoFoundationGrabberL");
        this.m_FoundationGrabberR = map.servo.get("servoFoundationGrabberR");
        SensorUtil.setServoPulseWidth(this.m_FoundationGrabberL,SwanSilverSettings.FOUNDATION_GRABBER_TYPE);
        SensorUtil.setServoPulseWidth(this.m_FoundationGrabberR,SwanSilverSettings.FOUNDATION_GRABBER_TYPE);
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
    public void updateTelemetry() {
        Telemetry globalTele = GlobalUtil.getTelemetry();
        if(globalTele != null){
            RobotPose2D currentPos = this.m_PosTracker.getCurrentPosition();
            {
                Telemetry.Line positionLine = globalTele.addLine("Current Position");
                positionLine.addData("X", currentPos.X);
                positionLine.addData("Y", currentPos.Y);
                positionLine.addData("RotZ", currentPos.getRotationZ());
            }
            {
                if (this.getChassis() != null && this.getChassis().isBusy()) {
                    RobotPose2D lastSupposedPose = this.getChassis().getCurrentTask().getLastSupposedPose();
                    Telemetry.Line lastSupposedPoseLine = globalTele.addLine("Last Supposed Pose");
                    lastSupposedPoseLine.addData("X",lastSupposedPose.X);
                    lastSupposedPoseLine.addData("Y",lastSupposedPose.Y);
                    lastSupposedPoseLine.addData("RotZ",lastSupposedPose.getRotationZ());
                }
            }
            {
                if(this.getChassis() != null && this.getChassis().isBusy()){
                    RobotPose2D lastError = this.getChassis().getCurrentTask().getLastError();
                    if(lastError != null){
                        Telemetry.Line lastErrorLine = globalTele.addLine("Last Error");
                        lastErrorLine.addData("X",lastError.X);
                        lastErrorLine.addData("Y",lastError.Y);
                        lastErrorLine.addData("RotZ",lastError.getRotationZ());
                    }
                }
            }
        }
        {
            Telemetry.Line slideLine = globalTele.addLine();
            slideLine.addData("Linear Slide","" + this.Slide.getCurrentPosition() + " (" + this.Slide.getCurrentPercent() + "%)");
        }
    }

    @Override
    public boolean isBusy() {
        return false;
    }

    @Override
    protected void __updateStatus() {
        this.m_Chassis.updateStatus();
        this.Slide.updateStatus();
        if(this.getGyro() != null && this.getGyro() instanceof RobotNonBlockingDevice){
            ((RobotNonBlockingDevice) this.getGyro()).updateStatus();
        }
    }

    public void setGrabberToGrab(boolean isGrabbing){
        if(isGrabbing){
            this.m_Grabber.setPosition(SwanSilverSettings.GRABBER_GRAB);
        }else{
            this.m_Grabber.setPosition(SwanSilverSettings.GRABBER_NOTGRAB);
        }
    }
}
