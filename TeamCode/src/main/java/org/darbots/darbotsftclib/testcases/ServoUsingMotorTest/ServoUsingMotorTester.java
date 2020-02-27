package org.darbots.darbotsftclib.testcases.ServoUsingMotorTest;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.darbots.darbotsftclib.libcore.OpModes.DarbotsBasicOpMode;
import org.darbots.darbotsftclib.libcore.tasks.servo_tasks.motor_powered_servo_tasks.TargetPosSpeedCtlTask;
import org.darbots.darbotsftclib.libcore.tasks.servo_tasks.motor_powered_servo_tasks.TargetPosTask;

@TeleOp(group = "DarbotsLib-TestCases",name="ServoUsingMotorTester")
@Disabled
public class ServoUsingMotorTester extends DarbotsBasicOpMode<ServoUsingMotorCore> {
    private static final double SERVOUSINGMOTOR_SPEED = 0.2;

    private ServoUsingMotorCore m_RobotCore;

    @Override
    public ServoUsingMotorCore getRobotCore() {
        return m_RobotCore;
    }

    @Override
    public void hardwareInitialize() {
        this.m_RobotCore = new ServoUsingMotorCore(this.hardwareMap);
    }

    @Override
    public void hardwareDestroy() {
        this.m_RobotCore = null;
    }

    @Override
    public void RunThisOpMode() {
        while(this.opModeIsActive()){
            telemetry.addData("CurrentPosition",this.m_RobotCore.getServoUsingMotor().getCurrentPosition());
            telemetry.update();
            this.m_RobotCore.updateStatus();
            if(this.m_RobotCore.isBusy()){
                continue;
            }
            if(gamepad1.dpad_down){
                this.m_RobotCore.getServoUsingMotor().replaceTask(new TargetPosTask(
                        null,0,SERVOUSINGMOTOR_SPEED
                ));
            }else if(gamepad1.dpad_up){
                this.m_RobotCore.getServoUsingMotor().replaceTask(new TargetPosTask(
                        null,1.0,SERVOUSINGMOTOR_SPEED
                ));
            }else if(gamepad1.dpad_left){
                this.m_RobotCore.getServoUsingMotor().replaceTask(new TargetPosSpeedCtlTask(
                        null,0,SERVOUSINGMOTOR_SPEED
                ));
            }else if(gamepad1.dpad_right){
                this.m_RobotCore.getServoUsingMotor().replaceTask(new TargetPosSpeedCtlTask(
                        null, 1.0, SERVOUSINGMOTOR_SPEED
                ));
            }
        }
    }
}
