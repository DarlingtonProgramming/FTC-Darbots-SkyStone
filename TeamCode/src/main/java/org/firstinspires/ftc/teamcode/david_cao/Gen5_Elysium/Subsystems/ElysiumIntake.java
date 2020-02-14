package org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.Subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.darbots.darbotsftclib.libcore.runtime.GlobalUtil;
import org.darbots.darbotsftclib.libcore.templates.RobotNonBlockingDevice;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ElysiumIntake implements RobotNonBlockingDevice {
    public static enum ElysiumIntakeStatus{
        SUCKING,
        SPITTING,
        STOPPED
    }
    private DcMotor intakeLeft;
    private DcMotor intakeRight;

    private ElysiumIntakeStatus lastStatus;
    private double lastSpeed;
    public ElysiumIntake(HardwareMap map){
        this.intakeLeft = map.dcMotor.get("intakeLeftMotor");
        this.intakeLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.intakeLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.intakeRight = map.dcMotor.get("intakeRightMotor");
        this.intakeRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.intakeRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public ElysiumIntakeStatus getIntakeSystemStatus(){
        return lastStatus;
    }

    public double getIntakeSystemSpeed(){
        return lastSpeed;
    }

    public void setIntakeSystemStatus(ElysiumIntakeStatus status, double speed){
        if(status == ElysiumIntakeStatus.STOPPED){
            lastSpeed = 0;
            this.intakeLeft.setPower(0);
            this.intakeRight.setPower(0);
        }else{
            lastSpeed = Math.abs(speed);
            if(status == ElysiumIntakeStatus.SUCKING){
                this.intakeLeft.setPower(lastSpeed);
                this.intakeRight.setPower(lastSpeed);
            }else{
                this.intakeLeft.setPower(-lastSpeed);
                this.intakeRight.setPower(-lastSpeed);
            }
        }
        lastStatus = status;
    }

    @Override
    public boolean isBusy() {
        return false;
    }

    @Override
    public void updateStatus() {
        return;
    }

    public void stop(){
        this.setIntakeSystemStatus(ElysiumIntakeStatus.STOPPED,0);
    }

    @Override
    public void waitUntilFinish() {
        return;
    }

    public void updateTelemetry(Telemetry telemetry, TelemetryPacket telemetryPacket){
        GlobalUtil.addTelmetryLine(telemetry,telemetryPacket,"Intake",this.getIntakeSystemStatus().name());
    }
}
