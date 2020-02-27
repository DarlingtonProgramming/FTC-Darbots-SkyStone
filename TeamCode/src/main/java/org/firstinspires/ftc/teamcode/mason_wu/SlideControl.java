package org.firstinspires.ftc.teamcode.mason_wu;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.darbots.darbotsftclib.libcore.motortypes.GoBilda5202Series117RPMMotor;

import org.darbots.darbotsftclib.libcore.templates.motor_related.MotorType;


@TeleOp(name="TeleOp-Slide", group="OpMode")
public class SlideControl extends LinearOpMode {

    final int max = 0;
    final int min = 0;
    private DcMotor slideMotor = hardwareMap.get(DcMotor.class, "stackerSlideMotor");
    private MotorType type = new GoBilda5202Series117RPMMotor();
    Slide slide = new Slide(slideMotor, type);


    @Override
    public void runOpMode() {

        slide.setMaxTick(max);
        slide.setMinTick(min);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Tick: " , slideMotor.getCurrentPosition());
            telemetry.update();
        }
    }


}
