package org.firstinspires.ftc.teamcode.mason_wu;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.darbots.darbotsftclib.libcore.motortypes.GoBilda5202Series1150RPMMotor;
import org.darbots.darbotsftclib.libcore.motortypes.GoBilda5202Series117RPMMotor;

import org.darbots.darbotsftclib.libcore.motortypes.MotorTypeUtil;
import org.darbots.darbotsftclib.libcore.templates.motor_related.MotorType;


@TeleOp(name="TeleOp-Chassis", group="OpMode")
public class ChassisControl extends LinearOpMode{
    public static final MotorType CHASSIS_MOTOR_TYPE = MotorTypeUtil.applyGearRatio(new GoBilda5202Series1150RPMMotor(),2);

    private DcMotor LF;
    private DcMotor RF;
    private DcMotor LB;
    private DcMotor RB;
    private double xPower;
    private double yPower;
    private double rotPower;
    private final double CONTROLLER_DEAD_ZONE = 0.1;
    @Override
    public void runOpMode() {
        LF = hardwareMap.get(DcMotor.class, "LF");
        RF = hardwareMap.get(DcMotor.class, "RF");
        LB = hardwareMap.get(DcMotor.class, "LB");
        RB = hardwareMap.get(DcMotor.class, "RB");
        LF.setDirection(DcMotor.Direction.REVERSE);
        LB.setDirection(DcMotor.Direction.REVERSE);
        Chassis chassis = new Chassis(LF,RF,LB,RB);
        chassis.setType(CHASSIS_MOTOR_TYPE);

        waitForStart();

        while (opModeIsActive()) {
            yPower = gamepad1.left_stick_x;
            xPower = -gamepad1.left_stick_y;
            if(xPower < CONTROLLER_DEAD_ZONE && xPower > - CONTROLLER_DEAD_ZONE)
                xPower = 0;
            if(yPower < CONTROLLER_DEAD_ZONE && yPower > - CONTROLLER_DEAD_ZONE)
                yPower = 0;
            rotPower = -gamepad1.right_stick_x;
            chassis.setPower(xPower,yPower,rotPower);
            telemetry.addData("X Power: ", xPower);
            telemetry.addData("Y Power: ", yPower);
            telemetry.update();
        }
    }
}
