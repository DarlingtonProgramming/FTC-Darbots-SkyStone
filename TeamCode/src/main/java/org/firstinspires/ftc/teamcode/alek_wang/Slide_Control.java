package org.firstinspires.ftc.teamcode.alek_wang;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.darbots.darbotsftclib.libcore.motortypes.GoBilda5202Series1150RPMMotor;
import org.darbots.darbotsftclib.libcore.templates.motor_related.MotorType;

@TeleOp(name = "SlideControlALek", group = "OpMode")
public class Slide_Control extends LinearOpMode {
    private DcMotor slidemotor = null;
    private MotorType slidetype = new GoBilda5202Series1150RPMMotor();
    private int low, high;
    private int count;


    @Override
    public void runOpMode() throws InterruptedException {
        slidemotor = hardwareMap.get(DcMotor.class, "slidemotor");
        low = slidemotor.getCurrentPosition();
        high = low + count;


        while (opModeIsActive()){
            telemetry.addData("Ticks", slidemotor.getCurrentPosition());
            telemetry.update();

            if(gamepad1.left_stick_y != 0){
                slidemotor.setPower(gamepad1.left_stick_y * 0.3);
            }else{
                slidemotor.setPower(0);
            }
        }


    }
}
