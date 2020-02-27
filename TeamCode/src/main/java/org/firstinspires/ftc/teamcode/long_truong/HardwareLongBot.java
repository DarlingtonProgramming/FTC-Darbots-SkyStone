package org.firstinspires.ftc.teamcode.long_truong;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class HardwareLongBot {
    public DcMotor flDrive;
    public DcMotor frDrive;
    public DcMotor blDrive;
    public DcMotor brDrive;
    public DcMotor pullMotor;
//    public Servo blservo;
//    public Servo brservo;
    public Servo grabservo;

    double power = 0.50;

    HardwareMap lhwmap;
    public ElapsedTime runtime = new ElapsedTime();

    public HardwareLongBot(){

    }

    public void init(HardwareMap lhwMap){

        flDrive = lhwMap.get(DcMotor.class, "fld");
        frDrive = lhwMap.get(DcMotor.class, "frd");
        blDrive = lhwMap.get(DcMotor.class, "bld");
        brDrive = lhwMap.get(DcMotor.class, "brd");
        pullMotor = lhwMap.get(DcMotor.class, "pm");
//        blservo = lhwMap.servo.get("bls");
//        brservo = lhwMap.servo.get("brs");
        grabservo = lhwMap.servo.get("gs");

        pullMotor.setDirection(DcMotor.Direction.FORWARD);
//        blservo.setDirection(Servo.Direction.FORWARD);
//        brservo.setDirection(Servo.Direction.REVERSE);
        grabservo.setDirection(Servo.Direction.REVERSE);

//        blservo.setPosition(servoPosition*0);
//        brservo.setPosition(servoPosition*0);
        grabservo.setPosition(0);

        flDrive.setPower(power*0);
        frDrive.setPower(power*0);
        blDrive.setPower(power*0);
        brDrive.setPower(power*0);
        pullMotor.setPower(power*0);

//        flDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        frDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        blDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        brDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
}
