package org.firstinspires.ftc.teamcode.david_cao.Gen2_Lindel_Code;

import android.hardware.Sensor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPose2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotVector2D;
import org.darbots.darbotsftclib.libcore.integratedfunctions.FTCMemory;
import org.darbots.darbotsftclib.libcore.motionsystems.MecanumDrivetrain;
import org.darbots.darbotsftclib.libcore.motortypes.AndyMark3637;
import org.darbots.darbotsftclib.libcore.odometry.MecanumChassis2DPositionTracker;
import org.darbots.darbotsftclib.libcore.runtime.GlobalUtil;
import org.darbots.darbotsftclib.libcore.runtime.SensorUtil;
import org.darbots.darbotsftclib.libcore.sensors.motion_related.RobotMotion;
import org.darbots.darbotsftclib.libcore.sensors.motion_related.RobotWheel;
import org.darbots.darbotsftclib.libcore.sensors.motors.RobotMotorController;
import org.darbots.darbotsftclib.libcore.sensors.motors.RobotMotorWithEncoder;
import org.darbots.darbotsftclib.libcore.sensors.servos.TimeControlledServo;
import org.darbots.darbotsftclib.libcore.sensors.servos.motor_powered_servos.RobotServoUsingMotor;
import org.darbots.darbotsftclib.libcore.tasks.servo_tasks.motor_powered_servo_tasks.TargetPosTask;
import org.darbots.darbotsftclib.libcore.templates.RobotCore;
import org.darbots.darbotsftclib.libcore.templates.RobotNonBlockingDevice;
import org.darbots.darbotsftclib.libcore.templates.chassis_related.RobotMotionSystem;
import org.darbots.darbotsftclib.libcore.templates.motor_related.MotorType;
import org.darbots.darbotsftclib.libcore.templates.motor_related.RobotMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LindelCore extends RobotCore {
    public enum IntakeSystemStatus{
        SUCK,
        VOMIT,
        STOP
    }
    private Servo m_DragServoL, m_DragServoR;
    private Servo m_Grabber;
    private TimeControlledServo m_GrabberRot;
    private Servo m_StoneOrientServo;
    private RobotServoUsingMotor m_linearSlide;
    private DcMotor m_IntakeLeft, m_IntakeRight;
    private Servo m_AutoDragStoneServoLeft, m_AutoDragStoneServoRight;
    private Servo m_CapStoneServo;

    private MecanumDrivetrain m_Chassis;
    private MecanumChassis2DPositionTracker m_PosTracker;

    public LindelCore(HardwareMap hardwareMap, String logFileName) {
        super(logFileName, hardwareMap);
        __initHardware(hardwareMap);
    }

    public LindelCore(HardwareMap hardwareMap, String logFileName, int ThreadPriority) {
        super(logFileName, hardwareMap, ThreadPriority);
        __initHardware(hardwareMap);
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
        RobotWheel m_LeftTopWheel = new RobotWheel(new RobotPose2D(LindelSettings.CHASSIS_LENGTH / 2,LindelSettings.CHASSIS_WIDTH / 2,45),LindelSettings.CHASSIS_WHEEL_RADIUS);
        RobotWheel m_RightTopWheel = new RobotWheel(new RobotPose2D(LindelSettings.CHASSIS_LENGTH / 2,-LindelSettings.CHASSIS_WIDTH / 2,-45),LindelSettings.CHASSIS_WHEEL_RADIUS);
        RobotWheel m_LeftBottomWheel = new RobotWheel(new RobotPose2D(LindelSettings.CHASSIS_LENGTH / 2,-LindelSettings.CHASSIS_WIDTH / 2,135),LindelSettings.CHASSIS_WHEEL_RADIUS);
        RobotWheel m_RightBottomWheel = new RobotWheel(new RobotPose2D(-LindelSettings.CHASSIS_LENGTH / 2,-LindelSettings.CHASSIS_WIDTH / 2,-135),LindelSettings.CHASSIS_WHEEL_RADIUS);
        RobotMotion LTMotion = new RobotMotion(new RobotMotorWithEncoder(m_LeftTopDC,LindelSettings.CHASSIS_MOTOR_TYPE),m_LeftTopWheel);
        RobotMotion LBMotion = new RobotMotion(new RobotMotorWithEncoder(m_LeftBottomDC,LindelSettings.CHASSIS_MOTOR_TYPE),m_LeftBottomWheel);
        RobotMotion RTMotion = new RobotMotion(new RobotMotorWithEncoder(m_RightTopDC,LindelSettings.CHASSIS_MOTOR_TYPE),m_RightTopWheel);
        RobotMotion RBMotion = new RobotMotion(new RobotMotorWithEncoder(m_RightBottomDC,LindelSettings.CHASSIS_MOTOR_TYPE),m_RightBottomWheel);
        
        this.m_Chassis = new MecanumDrivetrain(null,LTMotion,RTMotion,LBMotion,RBMotion);
        this.m_Chassis.setLinearXMotionDistanceFactor(LindelSettings.CHASSIS_FACTORS.X);
        this.m_Chassis.setLinearYMotionDistanceFactor(LindelSettings.CHASSIS_FACTORS.Y);
        this.m_Chassis.setRotationalMotionDistanceFactor(LindelSettings.CHASSIS_FACTORS.getRotationZ());
        this.m_PosTracker = new MecanumChassis2DPositionTracker(new RobotPose2D(0,0,0),this.m_Chassis);
        this.m_PosTracker.setDistanceFactors(new RobotVector2D(LindelSettings.ODOMETRY_FACTORS.X,LindelSettings.ODOMETRY_FACTORS.Y,LindelSettings.ODOMETRY_FACTORS.getRotationZ()));
        this.m_Chassis.setPositionTracker(this.m_PosTracker);
        this.m_PosTracker.start();

        this.m_DragServoL = map.servo.get("servoDragLeft");
        SensorUtil.setServoPulseWidth(this.m_DragServoL,LindelSettings.FOUNDATION_GRABBER_TYPE);
        this.m_DragServoR = map.servo.get("servoDragRight");
        SensorUtil.setServoPulseWidth(this.m_DragServoR,LindelSettings.FOUNDATION_GRABBER_TYPE);
        this.m_Grabber = map.servo.get("servoGrabber");
        SensorUtil.setServoPulseWidth(this.m_Grabber,LindelSettings.GRABBER_SERVO_TYPE);

        Servo GrabberRotServo = map.servo.get("servoGrabberRot");
        SensorUtil.setServoPulseWidth(GrabberRotServo,LindelSettings.GRABBERROT_SERVO_TYPE);
        this.m_GrabberRot = new TimeControlledServo(GrabberRotServo, LindelSettings.GRABBERROT_SERVO_TYPE, LindelSettings.GRABBERROT_IN,false);


        RobotMotor LinearSlideMotor = new RobotMotorWithEncoder(map.dcMotor.get("motorLinearSlide"), LindelSettings.LINEAR_SLIDE_MOTORTYPE);
        LinearSlideMotor.setDirectionReversed(true);
        RobotMotorController linearSlideController = new RobotMotorController(LinearSlideMotor, LindelSettings.LINEAR_SLIDE_TIMEOUT, LindelSettings.LINEAR_SLIDE_TIMEOUT_FACTOR);
        this.m_linearSlide = new RobotServoUsingMotor(linearSlideController, LindelSettings.LINEAR_SLIDE_INIT, LindelSettings.LINEAR_SLIDE_MIN, LindelSettings.LINEAR_SLIDE_MAX);

        this.m_IntakeLeft = map.dcMotor.get("motorIntakeLeft");
        this.m_IntakeRight = map.dcMotor.get("motorIntakeRight");

        this.m_StoneOrientServo = map.servo.get("servoStoneOrient");
        SensorUtil.setServoPulseWidth(this.m_StoneOrientServo,LindelSettings.STONE_ORIENT_SERVO_TYPE);

        this.m_AutoDragStoneServoLeft = map.servo.get("servoAutoDragStoneLeft");
        SensorUtil.setServoPulseWidth(this.m_AutoDragStoneServoLeft,LindelSettings.AUTONOMOUSDRAGSTONESERVO_TYPE);
        this.m_AutoDragStoneServoRight = map.servo.get("servoAutoDragStoneRight");
        SensorUtil.setServoPulseWidth(this.m_AutoDragStoneServoRight,LindelSettings.AUTONOMOUSDRAGSTONESERVO_TYPE);

        this.m_CapStoneServo = map.servo.get("servoCapStone");
        SensorUtil.setServoPulseWidth(this.m_CapStoneServo,LindelSettings.CAPSTONESERVO_SERVO_TYPE);

        this.setCapStoneServoToDeposit(false);
        this.setOrientServoToOrient(false);
        this.m_DragServoL.setPosition(LindelSettings.FOUNDATION_GRABBER_L_INIT);
        this.m_DragServoR.setPosition(LindelSettings.FOUNDATION_GRABBER_R_INIT);
        this.setAutonomousDragStoneServoLeftToDrag(false);
        this.setAutonomousDragStoneServoRightToDrag(false);
    }

    public void readAll(){
        this.readPosition();
        this.readSlidePosition();
        this.readGrabberRotPosition();
    }

    public void saveAll(){
        this.savePosition();
        this.saveSlidePosition();
        this.saveGrabberRotPosition();
    }

    public void readPosition(){
        RobotPose2D readPosition = FTCMemory.getSetting("LindelSlidePosition",this.getChassis().getPositionTracker().getCurrentPosition());
        if(readPosition != null) {
            this.m_PosTracker.setCurrentPosition(readPosition);
        }
    }

    public void savePosition(){
        FTCMemory.setSetting("LindelPosition",this.getChassis().getPositionTracker().getCurrentPosition());
        FTCMemory.saveMemoryMapToFile();
    }

    public void saveSlidePosition(){
        FTCMemory.setSetting("LindelSlidePosition",this.getLinearSlide().getCurrentPosition());
        FTCMemory.saveMemoryMapToFile();
    }

    public void readSlidePosition(){
        Double readPosition = FTCMemory.getSetting("LindelSlidePosition",this.getLinearSlide().getCurrentPosition());
        this.getLinearSlide().adjustCurrentPosition(readPosition);
    }

    public void saveGrabberRotPosition(){
        FTCMemory.setSetting("LindelGrabberRotPosition", this.m_GrabberRot.getCurrentPosition());
        FTCMemory.saveMemoryMapToFile();
    }

    public void readGrabberRotPosition(){
        Double readPosition = FTCMemory.getSetting("LindelGrabberRotPosition", this.m_GrabberRot.getCurrentPosition());
        this.m_GrabberRot.adjustLastPosition(readPosition);
    }

    public void setAutonomousDragStoneServoLeftToDrag(boolean toDrag){
        if(toDrag){
            this.m_AutoDragStoneServoLeft.setPosition(LindelSettings.AUTONOMOUSDRAGSTONESERVO_LEFT_OUTPOS);
        }else{
            this.m_AutoDragStoneServoLeft.setPosition(LindelSettings.AUTONOMOUSDRAGSTONESERVO_LEFT_INPOS);
        }
    }
    public void setAutonomousDragStoneServoRightToDrag(boolean toDrag){
        if(toDrag){
            this.m_AutoDragStoneServoRight.setPosition(LindelSettings.AUTONOMOUSDRAGSTONESERVO_RIGHT_OUTPOS);
        }else{
            this.m_AutoDragStoneServoRight.setPosition(LindelSettings.AUTONOMOUSDRAGSTONESERVO_RIGHT_IN);
        }
    }

    public TimeControlledServo getGrabberRotServo(){
        return this.m_GrabberRot;
    }

    public void setCapStoneServoToDeposit(boolean toDeposit){
        if(toDeposit){
            this.m_CapStoneServo.setPosition(LindelSettings.CAPSTONESERVO_DEPOSITPOS);
        }else{
            this.m_CapStoneServo.setPosition(LindelSettings.CAPSTONESERVO_INITIALPOS);
        }
    }

    public RobotServoUsingMotor getLinearSlide(){
        return this.m_linearSlide;
    }

    public void setLinearSlideToRecieveStonePos(double speed){
        this.getLinearSlide().replaceTask(new TargetPosTask(null, LindelSettings.LINEAR_SLIDE_INIT,speed));
    }
    public void setDragServoToDrag(boolean drag){
        if(drag){
            this.m_DragServoL.setPosition(LindelSettings.FOUNDATION_GRABBER_L_GRAB);
            this.m_DragServoR.setPosition(LindelSettings.FOUNDATION_GRABBER_R_GRAB);
        }else{
            this.m_DragServoL.setPosition(LindelSettings.FOUNDATION_GRABBER_L_RELEASED);
            this.m_DragServoR.setPosition(LindelSettings.FOUNDATION_GRABBER_R_RELEASED);
        }
    }
    public void setDragServoToInit(){
        this.m_DragServoL.setPosition(LindelSettings.FOUNDATION_GRABBER_L_INIT);
        this.m_DragServoR.setPosition(LindelSettings.FOUNDATION_GRABBER_R_INIT);
    }

    public void setGrabberServoToGrab(boolean grab){
        if(grab){
            this.m_Grabber.setPosition(LindelSettings.GRABBER_GRAB);
        }else{
            this.m_Grabber.setPosition(LindelSettings.GRABBER_NOTGRAB);
        }
    }

    public void setGrabberRotServoToOutside(boolean outside, double speed){
        if(outside){
            this.m_GrabberRot.setTargetPosition(LindelSettings.GRABBERROT_OUT, speed);
        }else{
            this.m_GrabberRot.setTargetPosition(LindelSettings.GRABBERROT_IN, speed);
        }
    }

    public void setIntakeSystemStatus(IntakeSystemStatus status, double Speed){
        Speed = Math.abs(Speed);
        switch(status){
            case SUCK:
                this.m_IntakeLeft.setPower(-Speed);
                this.m_IntakeRight.setPower(Speed);
                break;
            case VOMIT:
                this.m_IntakeLeft.setPower(Speed);
                this.m_IntakeRight.setPower(-Speed);
                break;
            case STOP:
                this.m_IntakeLeft.setPower(0);
                this.m_IntakeRight.setPower(0);
                break;
        }
    }

    public void setOrientServoToOrient(boolean orient){
        if(orient){
            this.m_StoneOrientServo.setPosition(LindelSettings.STONE_ORIENT_ORIENTPOS);
        }else{
            this.m_StoneOrientServo.setPosition(LindelSettings.STONE_ORIENT_ZEROPOS);
        }
    }

    public IntakeSystemStatus getIntakeSystemStatus(){
        if(this.m_IntakeLeft.getPower() < 0) {
            return IntakeSystemStatus.SUCK;
        }else if(this.m_IntakeLeft.getPower() > 0){
            return IntakeSystemStatus.VOMIT;
        }else{
            return IntakeSystemStatus.STOP;
        }
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
            {
                Telemetry.Line linearSlideLine = globalTele.addLine("Linear Slide");
                linearSlideLine.addData("Position",this.getLinearSlide().getCurrentPosition());
                linearSlideLine.addData("Percent",this.getLinearSlide().getCurrentPercent());
                linearSlideLine.addData("Min/Max","[" + this.getLinearSlide().getMinPos() + "," + this.getLinearSlide().getMaxPos() + "]");
            }
        }
    }

    @Override
    public boolean isBusy() {
        return this.m_linearSlide.isBusy() || this.m_GrabberRot.isBusy();
    }

    @Override
    protected void __updateStatus() {
        this.m_Chassis.updateStatus();
        this.m_linearSlide.updateStatus();
        this.m_GrabberRot.updateStatus();
        if(this.getGyro() != null && this.getGyro() instanceof RobotNonBlockingDevice){
            ((RobotNonBlockingDevice) this.getGyro()).updateStatus();
        }

    }
}
