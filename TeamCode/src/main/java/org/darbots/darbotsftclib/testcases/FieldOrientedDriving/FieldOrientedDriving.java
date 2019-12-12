package org.darbots.darbotsftclib.testcases.FieldOrientedDriving;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.darbots.darbotsftclib.libcore.OpModes.DarbotsBasicOpMode;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotVector2D;
import org.darbots.darbotsftclib.libcore.integratedfunctions.FieldOrientedMovementTeleOpControl;
import org.darbots.darbotsftclib.libcore.runtime.GlobalRegister;
import org.darbots.darbotsftclib.libcore.runtime.GlobalUtil;
import org.darbots.darbotsftclib.libcore.tasks.chassis_tasks.RobotMotionSystemTeleOpTask;
import org.darbots.darbotsftclib.libcore.templates.RobotNonBlockingDevice;
import org.darbots.darbotsftclib.testcases.common.TestMecanumCore;

@TeleOp(group = "DarbotsLib-TestCases", name = "FieldOrientedDrivingTest")
public class FieldOrientedDriving extends DarbotsBasicOpMode<TestMecanumCore> {
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
        RobotMotionSystemTeleOpTask teleOpTask = new RobotMotionSystemTeleOpTask();
        if(GlobalUtil.getGyro() != null && GlobalUtil.getGyro() instanceof RobotNonBlockingDevice) {
            ((RobotNonBlockingDevice) GlobalUtil.getGyro()).updateStatus();
        }
        float currentAngle = GlobalUtil.getGyro().getHeading();

        FieldOrientedMovementTeleOpControl controlFeeder = new FieldOrientedMovementTeleOpControl(currentAngle) {
            @Override
            public float getCurrentAngleCW() {
                return GlobalUtil.getGyro().getHeading();
            }
        };
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
            RobotVector2D convertedRobotSpeed = controlFeeder.getRobotSpeed(new RobotVector2D(normalizedXSpeed,normalizedYSpeed,normalizedRotSpeed));
            teleOpTask.xSpeedNormalized = convertedRobotSpeed.X;
            teleOpTask.ySpeedNormalized = convertedRobotSpeed.Y;
            teleOpTask.zRotSpeedNormalized = convertedRobotSpeed.getRotationZ();
            this.getRobotCore().updateTelemetry();
            telemetry.update();
            this.getRobotCore().updateStatus();
        }
    }
}
