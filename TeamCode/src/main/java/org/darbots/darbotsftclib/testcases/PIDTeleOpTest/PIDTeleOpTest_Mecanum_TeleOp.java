package org.darbots.darbotsftclib.testcases.PIDTeleOpTest;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.darbots.darbotsftclib.libcore_4_5_0Pre.OpModes.DarbotsBasicOpMode;
import org.darbots.darbotsftclib.libcore_4_5_0Pre.tasks.chassis_tasks.RobotMotionSystemTeleOpPIDTask;
import org.darbots.darbotsftclib.libcore_4_5_0Pre.tasks.chassis_tasks.RobotMotionSystemTeleOpTask;
import org.darbots.darbotsftclib.testcases.common.TestMecanumCore;

@TeleOp(group = "DarbotsLib-TestCases", name = "PIDTeleOpTest")
@Disabled
public class PIDTeleOpTest_Mecanum_TeleOp extends DarbotsBasicOpMode<TestMecanumCore> {
    private TestMecanumCore m_Core;

    @Override
    public TestMecanumCore getRobotCore() {
        return this.m_Core;
    }

    @Override
    public void hardwareInitialize() {
        this.m_Core = new TestMecanumCore(this.hardwareMap,"MecanumChassisControllerTest.log");
    }

    @Override
    public void hardwareDestroy() {

    }

    @Override
    public void RunThisOpMode() {
        RobotMotionSystemTeleOpTask teleOpTask = new RobotMotionSystemTeleOpPIDTask();
        this.getRobotCore().getChassis().addTask(teleOpTask);
        while(this.opModeIsActive()){
            double normalizedXSpeed = -this.gamepad1.left_stick_y;
            double normalizedYSpeed = -this.gamepad1.left_stick_x;
            double normalizedRotSpeed = -this.gamepad1.right_stick_x;
            if(Math.abs(normalizedXSpeed) < 0.15){
                normalizedXSpeed = 0;
            }
            if(Math.abs(normalizedYSpeed) < 0.15){
                normalizedYSpeed = 0;
            }
            if(Math.abs(normalizedRotSpeed) < 0.15){
                normalizedRotSpeed = 0;
            }
            teleOpTask.xSpeedNormalized = normalizedXSpeed;
            teleOpTask.ySpeedNormalized = normalizedYSpeed;
            teleOpTask.zRotSpeedNormalized = normalizedRotSpeed;
            this.getRobotCore().updateTelemetry();
            telemetry.update();
            this.getRobotCore().updateStatus();
        }
    }
}
