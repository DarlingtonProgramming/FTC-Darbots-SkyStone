package org.firstinspires.ftc.teamcode.mason_wu;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.darbots.darbotsftclib.libcore.templates.motor_related.MotorType;

public class Slide {
    int maxTick;
    int minTick;
    int relative;
    int currentTick;
    MotorType type;
    DcMotor motor;

    public Slide(){}

    public Slide(DcMotor motor, MotorType type){
        this.type = type;
        this.motor = motor;
        currentTick = motor.getCurrentPosition();
        minTick = currentTick;

    }

    public void setMaxTick (int max){
        maxTick = max;
    }
    public void setMinTick (int min){
        minTick = min;
    }
    public void setRelative (int relative){
        this.relative = relative;
    }
    public int getMaxTick (){
        return maxTick;
    }
    public int getMinTick (int min){
        return minTick;
    }

    public void runToTop(int power){
        motor.setTargetPosition(maxTick);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(power);

    }

    public void runToBottom(int power){
        motor.setTargetPosition(minTick);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(power);

    }

    public void runToPosition(int position, int power){
        motor.setTargetPosition(position);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(power);

    }

    public void runWithPower(double power){
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if(currentTick < maxTick && currentTick > minTick)
            motor.setPower(power);
        else {
            if(currentTick > maxTick && power < 0)
                motor.setPower(power);
            else if(currentTick < minTick && power > 0)
                motor.setPower(power);
            else
                motor.setPower(0);
        }
        currentTick = motor.getCurrentPosition();
    }
}
