package org.firstinspires.ftc.teamcode.alek_wang;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.darbots.darbotsftclib.libcore.motortypes.GoBilda5202Series1150RPMMotor;
import org.darbots.darbotsftclib.libcore.templates.motor_related.MotorType;

@TeleOp(name = "SlideControlALek", group = "OpMode")
public class Slide_Control extends LinearOpMode {
    private DcMotor slidemotor = null;
    private MotorType slidetype = new GoBilda5202Series1150RPMMotor();
    private int low, high;
    private int count = 1900;


    @Override
    public void runOpMode() throws InterruptedException {
        slidemotor = hardwareMap.get(DcMotor.class, "stackerSlideMotor");

        slidemotor.setDirection(DcMotor.Direction.REVERSE);
        slidemotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        low = slidemotor.getCurrentPosition();
        high = low + count;

        waitForStart();


        while (opModeIsActive()){
            telemetry.addData("Ticks", slidemotor.getCurrentPosition());
            telemetry.update();

            double power = -gamepad1.left_stick_y;

            if(power != 0){
                if(slidemotor.getCurrentPosition() >= low && slidemotor.getCurrentPosition() <= high){
                    slidemotor.setPower(power * 0.8);
                }
                else if(slidemotor.getCurrentPosition() < low && power > 0){
                    slidemotor.setPower(power);

                }
                else if(slidemotor.getCurrentPosition() > high && power < 0){
                    slidemotor.setPower(power);

                }else{
                    slidemotor.setPower(0);
                }
            }else{
                slidemotor.setPower(0);
            }
        }


    }
}
