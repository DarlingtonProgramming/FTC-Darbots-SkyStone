package org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.Subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.darbots.darbotsftclib.libcore.runtime.GlobalUtil;
import org.darbots.darbotsftclib.libcore.runtime.SensorUtil;
import org.darbots.darbotsftclib.libcore.templates.RobotNonBlockingDevice;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.Elysium_Settings.ElysiumSettings;

public class ElysiumIntake implements RobotNonBlockingDevice {
    public static enum ElysiumIntakeStatus{
        SUCKING,
        SPITTING,
        STOPPED
    }
    public static enum ElysiumIntakePositioningServoStatus{
        REST,
        AUTO,
        HIT
    }
    private DcMotor intakeLeft;
    private DcMotor intakeRight;
    private Servo intakeOrient;


    private ElysiumIntakeStatus lastStatus = ElysiumIntakeStatus.STOPPED;
    private double lastSpeed = 0;
    public ElysiumIntake(HardwareMap map){
        this.intakeLeft = map.dcMotor.get("intakeLeftMotor");
        this.intakeLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.intakeLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.intakeRight = map.dcMotor.get("intakeRightMotor");
        this.intakeRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.intakeRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.intakeOrient = map.servo.get("intakePositioningServo");
        SensorUtil.setServoPulseWidth(this.intakeOrient, ElysiumSettings.INTAKE_POSITIONING_SERVO_TYPE);
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
