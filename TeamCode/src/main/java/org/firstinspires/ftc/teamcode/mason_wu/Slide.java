package org.firstinspires.ftc.teamcode.mason_wu;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.darbots.darbotsftclib.libcore.templates.motor_related.MotorType;

public class Slide {
    int maxTick;
    int minTick;
    MotorType type;
    DcMotor motor;

    public Slide(){}

    public Slide(DcMotor motor, MotorType type){
        this.type = type;
        this.motor = motor;
    }

    public void setMaxTick (int max){
        maxTick = max;
    }
    public void setMinTick (int min){
        minTick = min;
    }
    public int getMaxTick (){
        return maxTick;
    }
    public int getMinTick (int min){
        return minTick;
    }

    public void runToTop(int power){
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(power);
        motor.setTargetPosition(maxTick);
    }

    public void runToBottom(int power){
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(power);
        motor.setTargetPosition(minTick);
    }

    public void runToPosition(int position, int power){
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(power);
        motor.setTargetPosition(position);
    }
}
