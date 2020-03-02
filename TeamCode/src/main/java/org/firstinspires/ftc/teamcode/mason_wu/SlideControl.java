package org.firstinspires.ftc.teamcode.mason_wu;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.darbots.darbotsftclib.libcore.motortypes.GoBilda5202Series117RPMMotor;

import org.darbots.darbotsftclib.libcore.templates.motor_related.MotorType;


@TeleOp(name="TeleOp-Slide", group="OpMode")
public class SlideControl extends LinearOpMode {

    int max = 2000;
    int min = 0;
    final int relative = max - min;
    private DcMotor slideMotor = null;
    private MotorType type = new GoBilda5202Series117RPMMotor();

    @Override
    public void runOpMode() {

        slideMotor = hardwareMap.get(DcMotor.class, "stackerSlideMotor");

        slideMotor.setDirection(DcMotor.Direction.REVERSE);
        Slide slide = new Slide(slideMotor, type);
        min = slideMotor.getCurrentPosition();
        max = min + relative;
        slide.setMaxTick(max);
        slide.setMinTick(min);
        slide.setRelative(relative);

        waitForStart();

        while (opModeIsActive()) {

            telemetry.addData("Tick: " , slideMotor.getCurrentPosition());
            telemetry.update();

            double power = -gamepad1.left_stick_y;
            slide.runWithPower(power);
        }
    }


}
