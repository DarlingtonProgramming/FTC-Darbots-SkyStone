package org.firstinspires.ftc.teamcode.mason_wu;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.darbots.darbotsftclib.libcore.sensors.motors.RobotMotorWithEncoder;
import org.darbots.darbotsftclib.libcore.templates.RobotCore;
import org.darbots.darbotsftclib.libcore.templates.motor_related.MotorType;
import org.darbots.darbotsftclib.libcore.templates.motor_related.RobotMotor;

@TeleOp(name="TeleOp-Slide", group="OpMode")
public class SlideControl extends LinearOpMode {
    private DcMotor LinearSlide = hardwareMap.get(DcMotor.class, "FL");
    RobotMotorWithEncoder Slide = new RobotMotorWithEncoder(LinearSlide, null);

    Slide.setDirectionReversed(true);
    Slide.setCurrentMovingType(MovingType.toCount);
    final int max = 0;
    final int min = 0;

    @Override
    public void runOpMode() {

        waitForStart();

        if (opModeIsActive()) {


        }
    }

}
