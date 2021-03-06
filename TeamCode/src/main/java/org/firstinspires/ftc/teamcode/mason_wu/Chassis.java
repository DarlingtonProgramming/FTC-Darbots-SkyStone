package org.firstinspires.ftc.teamcode.mason_wu;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.darbots.darbotsftclib.libcore.templates.motor_related.MotorType;

public class Chassis {
    private DcMotor LF;
    private DcMotor RF;
    private DcMotor LB;
    private DcMotor RB;


    MotorType type;

    public Chassis(){ }
    public Chassis(DcMotor LF, DcMotor RF, DcMotor LB, DcMotor RB){
        this.LF = LF;
        this.LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.RF = RF;
        this.RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.LB = LB;
        this.LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.RB = RB;
        this.RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    public void setType(MotorType type) {
        this.type = type;
    }

    public void setPower(double xPower, double yPower, double rotPower) {
        Range.clip(xPower, -1.0,1.0);
        Range.clip(yPower, -1.0,1.0);
        Range.clip(rotPower, -1.0,1.0);
        LF.setPower(xPower+yPower-rotPower);
        RF.setPower(xPower-yPower+rotPower);
        LB.setPower(xPower-yPower-rotPower);
        RB.setPower(xPower+yPower+rotPower);

    }

}
