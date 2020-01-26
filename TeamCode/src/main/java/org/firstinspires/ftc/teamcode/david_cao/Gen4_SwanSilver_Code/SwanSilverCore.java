package org.firstinspires.ftc.teamcode.david_cao.Gen4_SwanSilver_Code;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.darbots.darbotsftclib.libcore_4_5_0Pre.calculations.dimentional_calculation.RobotPose2D;
import org.darbots.darbotsftclib.libcore_4_5_0Pre.calculations.dimentional_calculation.RobotVector2D;
import org.darbots.darbotsftclib.libcore_4_5_0Pre.integratedfunctions.FTCMemory;
import org.darbots.darbotsftclib.libcore_4_5_0Pre.motionsystems.MecanumDrivetrain;
import org.darbots.darbotsftclib.libcore_4_5_0Pre.odometry.MecanumChassis2DPositionTracker;
import org.darbots.darbotsftclib.libcore_4_5_0Pre.runtime.GlobalUtil;
import org.darbots.darbotsftclib.libcore_4_5_0Pre.runtime.SensorUtil;
import org.darbots.darbotsftclib.libcore_4_5_0Pre.sensors.motion_related.RobotMotion;
import org.darbots.darbotsftclib.libcore_4_5_0Pre.sensors.motion_related.RobotWheel;
import org.darbots.darbotsftclib.libcore_4_5_0Pre.sensors.motors.RobotMotorController;
import org.darbots.darbotsftclib.libcore_4_5_0Pre.sensors.motors.RobotMotorWithEncoder;
import org.darbots.darbotsftclib.libcore_4_5_0Pre.sensors.servos.TimeControlledServo;
import org.darbots.darbotsftclib.libcore_4_5_0Pre.sensors.servos.motor_powered_servos.RobotServoUsingMotor;
import org.darbots.darbotsftclib.libcore_4_5_0Pre.templates.RobotCore;
import org.darbots.darbotsftclib.libcore_4_5_0Pre.templates.RobotNonBlockingDevice;
import org.darbots.darbotsftclib.libcore_4_5_0Pre.templates.chassis_related.RobotMotionSystem;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SwanSilverCore extends RobotCore {

    private MecanumDrivetrain m_Chassis;
    private MecanumChassis2DPositionTracker m_PosTracker;
    public RobotServoUsingMotor Slide;
    private Servo m_Grabber;
    public TimeControlledServo GrabberMover;
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
        FTCMemory.saveMemoryMapToFile();
    }

    public void saveSlidePosition(){
        FTCMemory.setSetting("SwanSilverSlidePos",this.Slide.getCurrentPosition());
        FTCMemory.saveMemoryMapToFile();
    }

    public void readSlidePosition(){
        Double readPosition = FTCMemory.getSetting("SwanSilverSlidePos",this.Slide.getCurrentPosition());
        this.Slide.adjustCurrentPosition(readPosition);
    }

    public void readAll(){
        this.readPosition();
        this.readSlidePosition();
    }

    public void saveAll(){
        this.savePosition();
        this.saveSlidePosition();
    }

    protected void __initHardware(HardwareMap map){
        DcMotor m_LeftTopDC = map.dcMotor.get("LF");
        DcMotor m_LeftBottomDC = map.dcMotor.get("LB");
        DcMotor m_RightTopDC = map.dcMotor.get("RF");
        DcMotor m_RightBottomDC = map.dcMotor.get("RB");

        m_LeftTopDC.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m_LeftBottomDC.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m_RightTopDC.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m_RightBottomDC.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
        Servo GrabberMoverServo = map.servo.get("servoGrabberMover");
        SensorUtil.setServoPulseWidth(GrabberMoverServo,SwanSilverSettings.GRABBERMOVER_SERVO_TYPE);
        this.GrabberMover = new TimeControlledServo(GrabberMoverServo,SwanSilverSettings.GRABBERMOVER_SERVO_TYPE,SwanSilverSettings.GRABBERMOVER_MEDIUM,true);

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
            {
                RobotPose2D currentPos = this.m_PosTracker.getCurrentPosition();
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
                RobotVector2D currentVelocity = this.m_PosTracker.getCurrentVelocityVector();
                Telemetry.Line velocityLine = globalTele.addLine("Current Velocity");
                double velocity = Math.sqrt(Math.pow(currentVelocity.X,2) + Math.pow(currentVelocity.Y,2));
                velocityLine.addData("Linear",velocity);
                velocityLine.addData("X",currentVelocity.X);
                velocityLine.addData("Y",currentVelocity.Y);
                velocityLine.addData("RotZ",currentVelocity.getRotationZ());
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
        this.GrabberMover.updateStatus();
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

    public void setFoundationGrabberToGrab(boolean isGrabbing){
        if(isGrabbing){
            this.m_FoundationGrabberL.setPosition(SwanSilverSettings.FOUNDATION_GRABBER_L_GRAB);
            this.m_FoundationGrabberR.setPosition(SwanSilverSettings.FOUNDATION_GRABBER_R_GRAB);
        }else{
            this.m_FoundationGrabberL.setPosition(SwanSilverSettings.FOUNDATION_GRABBER_L_RELEASED);
            this.m_FoundationGrabberR.setPosition(SwanSilverSettings.FOUNDATION_GRABBER_R_RELEASED);
        }
    }
}
