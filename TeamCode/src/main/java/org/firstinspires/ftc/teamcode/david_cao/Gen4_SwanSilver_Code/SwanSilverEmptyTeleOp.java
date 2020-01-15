package org.firstinspires.ftc.teamcode.david_cao.Gen4_SwanSilver_Code;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.darbots.darbotsftclib.libcore.OpModes.DarbotsBasicOpMode;
import org.darbots.darbotsftclib.libcore.tasks.chassis_tasks.RobotMotionSystemTeleOpTask;
import org.darbots.darbotsftclib.libcore.tasks.servo_tasks.motor_powered_servo_tasks.TargetPosSpeedCtlTask;
import org.darbots.darbotsftclib.libcore.templates.servo_related.motor_powered_servos.RobotServoUsingMotorTask;

@TeleOp(group = "4100", name = "4100TeleOpEmpty")
public class SwanSilverEmptyTeleOp extends DarbotsBasicOpMode<SwanSilverCore> {
    private SwanSilverCore m_Core;
    private RobotMotionSystemTeleOpTask driveTask;
    @Override
    public SwanSilverCore getRobotCore() {
        return this.m_Core;
    }

    @Override
    public void hardwareInitialize() {
        this.m_Core = new SwanSilverCore(this.hardwareMap,"SwanSilverEmptyTeleOp.log");
        driveTask = new RobotMotionSystemTeleOpTask();
        this.m_Core.getChassis().addTask(driveTask);
    }

    @Override
    public void hardwareDestroy() {

    }

    @Override
    public void RunThisOpMode() {
        while(this.opModeIsActive()){
            driveControl();
            slideControl();

            this.m_Core.updateStatus();
            this.m_Core.updateTelemetry();
            this.telemetry.update();
        }
    }

    protected void driveControl(){
        if(Math.abs(gamepad1.left_stick_x) >= SwanSilverSettings.CONTROL_DRIVE_THRESEHOLD || Math.abs(gamepad1.left_stick_y) >= SwanSilverSettings.CONTROL_DRIVE_THRESEHOLD || Math.abs(gamepad1.right_stick_x) >= SwanSilverSettings.CONTROL_DRIVE_THRESEHOLD){
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

    protected void slideControl(){
        if(Math.abs(gamepad2.left_stick_y) >= SwanSilverSettings.CONTROL_STICK_THRESHOLD){
            double targetY = -gamepad2.left_stick_y;
            RobotServoUsingMotorTask currentTask = this.m_Core.Slide.getCurrentTask();
            TargetPosSpeedCtlTask currentSpecificTask = currentTask == null || currentTask instanceof TargetPosSpeedCtlTask ? null : (TargetPosSpeedCtlTask) currentTask;
            if(targetY > 0){
                if(currentSpecificTask != null && currentSpecificTask.getTargetPos() == SwanSilverSettings.LINEAR_SLIDE_MAX){
                    currentSpecificTask.setPower(targetY);
                }else{
                    TargetPosSpeedCtlTask newTask = new TargetPosSpeedCtlTask(null,SwanSilverSettings.LINEAR_SLIDE_MAX,targetY);
                    this.m_Core.Slide.replaceTask(newTask);
                }

            }else{ //targetY < 0
                if(currentSpecificTask != null && currentSpecificTask.getTargetPos() == SwanSilverSettings.LINEAR_SLIDE_MIN){
                    currentSpecificTask.setPower(-targetY);
                }else{
                    TargetPosSpeedCtlTask newTask = new TargetPosSpeedCtlTask(null,SwanSilverSettings.LINEAR_SLIDE_MIN,-targetY);
                    this.m_Core.Slide.replaceTask(newTask);
                }
            }
        }else if(this.m_Core.Slide.isBusy()){
            this.m_Core.Slide.deleteAllTasks();
        }
    }
}
