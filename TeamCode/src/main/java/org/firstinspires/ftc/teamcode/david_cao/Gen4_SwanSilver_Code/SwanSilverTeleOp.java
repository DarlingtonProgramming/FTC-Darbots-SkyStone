package org.firstinspires.ftc.teamcode.david_cao.Gen4_SwanSilver_Code;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.darbots.darbotsftclib.libcore.OpModes.DarbotsBasicOpMode;
import org.darbots.darbotsftclib.libcore.tasks.chassis_tasks.RobotMotionSystemTeleOpTask;
import org.darbots.darbotsftclib.libcore.tasks.servo_tasks.motor_powered_servo_tasks.TargetPosSpeedCtlTask;
import org.darbots.darbotsftclib.libcore.templates.servo_related.motor_powered_servos.RobotServoUsingMotorTask;

@TeleOp(group = "4100", name = "4100Gen4-TeleOp")
public class SwanSilverTeleOp extends DarbotsBasicOpMode<SwanSilverCore> {
    private SwanSilverCore m_Core;
    private RobotMotionSystemTeleOpTask driveTask;
    @Override
    public SwanSilverCore getRobotCore() {
        return this.m_Core;
    }

    @Override
    public void hardwareInitialize() {
        this.m_Core = new SwanSilverCore(this.hardwareMap,"SwanSilverTeleOp.log");
        driveTask = new RobotMotionSystemTeleOpTask();
        this.m_Core.getChassis().addTask(driveTask);
        this.m_Core.GrabberMover.adjustLastPosition(SwanSilverSettings.GRABBERMOVER_MEDIUM);
    }

    @Override
    public void hardwareDestroy() {

    }

    @Override
    public void RunThisOpMode() {
        while(this.opModeIsActive()){
            driveControl();
            foundationGraberControl();
            slideControl();
            grabberControl();
            grabberMoverControl();

            this.m_Core.updateStatus();
            this.m_Core.updateTelemetry();
            this.telemetry.update();
        }
    }

    protected void driveControl(){
        if(Math.abs(gamepad1.left_stick_x) >= SwanSilverSettings.CONTROL_STICK_THRESHOLD || Math.abs(gamepad1.left_stick_y) >= SwanSilverSettings.CONTROL_STICK_THRESHOLD || Math.abs(gamepad1.right_stick_x) >= SwanSilverSettings.CONTROL_STICK_THRESHOLD){
            double xControl = -gamepad1.left_stick_y;
            double yControl = -gamepad1.left_stick_x;
            double rotControl = -gamepad1.right_stick_x;
            driveTask.xSpeedNormalized = xControl;
            driveTask.ySpeedNormalized = yControl;
            driveTask.zRotSpeedNormalized = rotControl;
        }else{
            driveTask.xSpeedNormalized = 0;
            driveTask.ySpeedNormalized = 0;
            driveTask.zRotSpeedNormalized = 0;
        }
    }

    protected void foundationGraberControl(){
        if(gamepad1.right_bumper){
            this.m_Core.setFoundationGrabberToGrab(true);
        }else{
            this.m_Core.setFoundationGrabberToGrab(false);
        }
    }

    protected void slideControl(){
        if(Math.abs(gamepad2.left_stick_y) >= SwanSilverSettings.CONTROL_STICK_THRESHOLD){
            double targetY = -gamepad2.left_stick_y;
            RobotServoUsingMotorTask currentTask = this.m_Core.Slide.getCurrentTask();
            TargetPosSpeedCtlTask currentSpecificTask = currentTask == null || !(currentTask instanceof TargetPosSpeedCtlTask) ? null : (TargetPosSpeedCtlTask) currentTask;
            if(targetY > 0){
                if(currentSpecificTask != null && currentSpecificTask.getTargetPos() == this.m_Core.Slide.getMaxPos()){
                    currentSpecificTask.setPower(targetY);
                }else{
                    TargetPosSpeedCtlTask newTask = new TargetPosSpeedCtlTask(null,this.m_Core.Slide.getMaxPos(),targetY);
                    this.m_Core.Slide.replaceTask(newTask);
                }
            }else{ //targetY < 0
                if(currentSpecificTask != null && currentSpecificTask.getTargetPos() == this.m_Core.Slide.getMinPos()){
                    currentSpecificTask.setPower(-targetY);
                }else{
                    TargetPosSpeedCtlTask newTask = new TargetPosSpeedCtlTask(null,this.m_Core.Slide.getMinPos(),-targetY);
                    this.m_Core.Slide.replaceTask(newTask);
                }
            }
        }else if(this.m_Core.Slide.isBusy()){
            this.m_Core.Slide.deleteAllTasks();
        }
    }
    protected void grabberControl(){
        if(this.gamepad2.right_bumper){
            this.m_Core.setGrabberToGrab(true);
        }else{
            this.m_Core.setGrabberToGrab(false);
        }
    }
    protected void grabberMoverControl(){
        if(Math.abs(this.gamepad2.right_stick_x) >= SwanSilverSettings.CONTROL_STICK_THRESHOLD){
            if(this.gamepad2.right_stick_x > 0) {
                this.m_Core.GrabberMover.setTargetPosition(SwanSilverSettings.GRABBERMOVER_MAX,this.gamepad2.right_stick_x);
            }else{
                this.m_Core.GrabberMover.setTargetPosition(SwanSilverSettings.GRABBERMOVER_MIN,-this.gamepad2.right_stick_x);
            }
        }else{
            this.m_Core.GrabberMover.stop();
        }
    }
}
