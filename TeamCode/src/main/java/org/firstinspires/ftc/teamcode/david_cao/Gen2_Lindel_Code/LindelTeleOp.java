package org.firstinspires.ftc.teamcode.david_cao.Gen2_Lindel_Code;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.darbots.darbotsftclib.libcore.OpModes.DarbotsBasicOpMode;
import org.darbots.darbotsftclib.libcore.tasks.chassis_tasks.RobotMotionSystemTeleOpTask;
import org.darbots.darbotsftclib.libcore.tasks.servo_tasks.motor_powered_servo_tasks.TargetPosSpeedCtlTask;
import org.darbots.darbotsftclib.libcore.templates.DarbotsComboKey;
import org.darbots.darbotsftclib.libcore.templates.servo_related.motor_powered_servos.RobotServoUsingMotorTask;

@TeleOp(group = "4100", name = "LindelTeleOp-Gen2.1")
public class LindelTeleOp extends DarbotsBasicOpMode<LindelCore> {
    private LindelCore m_Core;
    public double SpeedFactor = 1.0;
    private boolean keepSucking = true;
    private RobotMotionSystemTeleOpTask driveTask = null;
    private DarbotsComboKey stoneOrientCombo;
    private DarbotsComboKey capstoneCombo;

    @Override
    public LindelCore getRobotCore() {
        return m_Core;
    }

    @Override
    public void hardwareInitialize() {
        this.m_Core = new LindelCore(this.hardwareMap,"LindelGen2.1-TeleOp.log");
        driveTask = new RobotMotionSystemTeleOpTask();
        this.m_Core.getChassis().replaceTask(driveTask);
        stoneOrientCombo = new DarbotsComboKey() {
            ElapsedTime time;
            @Override
            protected void __startCombo() {
                time = new ElapsedTime();
                m_Core.setOrientServoToOrient(true);
            }

            @Override
            protected void __stopCombo() {
                m_Core.setOrientServoToOrient(false);
            }

            @Override
            public void updateStatus() {
                if(this.isBusy()) {
                    if (time.milliseconds() >= 500) {
                        m_Core.setOrientServoToOrient(false);
                        this.stopCombo();
                    }
                }
            }
        };
        capstoneCombo = new DarbotsComboKey() {
            ElapsedTime time;
            @Override
            protected void __startCombo() {
                m_Core.setCapStoneServoToDeposit(true);
                time = new ElapsedTime();
            }

            @Override
            protected void __stopCombo() {
                m_Core.setCapStoneServoToDeposit(false);
            }

            @Override
            public void updateStatus() {
                if(this.isBusy()){
                    if(time.milliseconds() >= 1000){
                        m_Core.setCapStoneServoToDeposit(false);
                        this.stopCombo();
                    }
                }
            }
        };
    }

    @Override
    public void hardwareDestroy() {
        this.stoneOrientCombo = null;
        this.capstoneCombo = null;
    }

    @Override
    public void RunThisOpMode() {
        waitForStart();
        while(this.opModeIsActive()) {
            driveControl();
            foundationGraberControl();
            slideControl();
            grabberControl();
            grabberRotControl();
            intakeControl();
            capstoneControl();
            orientServoControl();
            this.m_Core.updateStatus();
            this.m_Core.updateTelemetry();
            if(this.capstoneCombo.isBusy()){
                this.capstoneCombo.updateStatus();
            }
            if(this.stoneOrientCombo.isBusy()){
                this.stoneOrientCombo.updateStatus();
            }
            telemetry.update();
        }
    }

    protected void driveControl(){
        if(Math.abs(gamepad1.left_stick_x) >= LindelSettings.CONTROL_STICK_THRESHOLD || Math.abs(gamepad1.left_stick_y) >= LindelSettings.CONTROL_STICK_THRESHOLD || Math.abs(gamepad1.right_stick_x) >= LindelSettings.CONTROL_STICK_THRESHOLD){
            double xControl = -gamepad1.left_stick_y;
            double yControl = -gamepad1.left_stick_x;
            double rotControl = -gamepad1.right_stick_x;
            if(gamepad1.left_bumper){
                xControl *= 0.25;
                yControl *= 0.25;
                rotControl *= 0.25;
            }
            driveTask.xSpeedNormalized = xControl * LindelSettings.CONTROL_CHASSIS_MAXSPEED_NORMALIZED * this.SpeedFactor;
            driveTask.ySpeedNormalized = yControl * LindelSettings.CONTROL_CHASSIS_MAXSPEED_NORMALIZED * this.SpeedFactor;
            driveTask.zRotSpeedNormalized = rotControl * LindelSettings.CONTROL_CHASSIS_MAXSPEED_NORMALIZED * this.SpeedFactor;
        }else{
            driveTask.xSpeedNormalized = 0;
            driveTask.ySpeedNormalized = 0;
            driveTask.zRotSpeedNormalized = 0;
        }
    }

    protected void foundationGraberControl(){
        if(gamepad1.right_bumper){
            this.getRobotCore().setDragServoToDrag(true);
        }else{
            this.m_Core.setDragServoToDrag(false);
        }
    }

    protected void slideControl(){
        if(Math.abs(gamepad2.left_stick_y) >= LindelSettings.CONTROL_STICK_THRESHOLD){
            double targetY = -gamepad2.left_stick_y;
            RobotServoUsingMotorTask currentTask = this.m_Core.getLinearSlide().getCurrentTask();
            TargetPosSpeedCtlTask currentSpecificTask = currentTask == null || !(currentTask instanceof TargetPosSpeedCtlTask) ? null : (TargetPosSpeedCtlTask) currentTask;
            double slideSpeed = Math.abs(LindelSettings.CONTROL_SLIDE_MAXSPEED);
            if(targetY > 0){
                if(currentSpecificTask != null && currentSpecificTask.getTargetPos() == this.m_Core.getLinearSlide().getMaxPos()){
                    currentSpecificTask.setPower(slideSpeed);
                }else{
                    TargetPosSpeedCtlTask newTask = new TargetPosSpeedCtlTask(null,this.m_Core.getLinearSlide().getMaxPos(),slideSpeed);
                    this.m_Core.getLinearSlide().replaceTask(newTask);
                }
            }else{ //targetY < 0
                if(currentSpecificTask != null && currentSpecificTask.getTargetPos() == this.m_Core.getLinearSlide().getMinPos()){
                    currentSpecificTask.setPower(slideSpeed);
                }else{
                    TargetPosSpeedCtlTask newTask = new TargetPosSpeedCtlTask(null,this.m_Core.getLinearSlide().getMinPos(),slideSpeed);
                    this.m_Core.getLinearSlide().replaceTask(newTask);
                }
            }
        }else if(this.m_Core.getLinearSlide().isBusy()){
            this.m_Core.getLinearSlide().deleteAllTasks();
        }
    }

    protected void grabberControl(){
        if(gamepad2.right_bumper){
            this.m_Core.setGrabberServoToGrab(false);
        }else{
            this.m_Core.setGrabberServoToGrab(true);
        }
    }

    protected void grabberRotControl(){
        if(gamepad2.right_trigger >= LindelSettings.CONTROL_STICK_THRESHOLD){
            this.m_Core.setGrabberRotServoToOutside(true,1.0);
        }else if(gamepad2.left_trigger >= LindelSettings.CONTROL_STICK_THRESHOLD){
            this.m_Core.setGrabberRotServoToOutside(false,1.0);
        }
    }

    protected void intakeControl(){
        if(gamepad1.right_trigger >= LindelSettings.CONTROL_STICK_THRESHOLD){
            this.m_Core.setIntakeSystemStatus(LindelCore.IntakeSystemStatus.SUCK,gamepad1.right_trigger * LindelSettings.INTAKEMOTOR_SPEED);
            keepSucking = true;
        }else if(gamepad1.left_trigger >= LindelSettings.CONTROL_STICK_THRESHOLD){
            this.m_Core.setIntakeSystemStatus(LindelCore.IntakeSystemStatus.VOMIT,gamepad1.left_trigger * LindelSettings.INTAKEMOTOR_SPEED);
            keepSucking = true;
        }else{
            if(keepSucking){
                this.m_Core.setIntakeSystemStatus(LindelCore.IntakeSystemStatus.SUCK,gamepad1.right_trigger * LindelSettings.INTAKEMOTOR_SPEED);
            }else {
                this.m_Core.setIntakeSystemStatus(LindelCore.IntakeSystemStatus.STOP,0);
            }
        }
    }

    protected void capstoneControl(){
        if(gamepad2.x && (!capstoneCombo.isBusy())){
            this.capstoneCombo.startCombo();
        }
    }

    protected void orientServoControl(){
        if(gamepad2.a && (!stoneOrientCombo.isBusy())){
            this.stoneOrientCombo.startCombo();
        }
    }
}
