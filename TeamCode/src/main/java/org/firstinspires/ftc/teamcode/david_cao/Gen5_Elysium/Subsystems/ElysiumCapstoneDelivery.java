package org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.Subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.darbots.darbotsftclib.libcore.integratedfunctions.FTCMemory;
import org.darbots.darbotsftclib.libcore.runtime.GlobalUtil;
import org.darbots.darbotsftclib.libcore.runtime.SensorUtil;
import org.darbots.darbotsftclib.libcore.sensors.motors.RobotMotorController;
import org.darbots.darbotsftclib.libcore.sensors.motors.RobotMotorWithEncoder;
import org.darbots.darbotsftclib.libcore.sensors.servos.motor_powered_servos.RobotServoUsingMotor;
import org.darbots.darbotsftclib.libcore.templates.RobotNonBlockingDevice;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.Elysium_Settings.ElysiumSettings;

public class ElysiumCapstoneDelivery implements RobotNonBlockingDevice {
    public static final double CAPSTONEDELIVERY_SLIDE_MIN_POS = ElysiumSettings.CAPSTONE_SLIDE_MIN;
    public static final double CAPSTONEDELIVERY_SLIDE_MAX_POS = ElysiumSettings.CAPSTONE_SLIDE_MAX;

    public RobotServoUsingMotor capstoneSlide;
    private Servo capstoneRotServo;
    public ElysiumCapstoneDelivery(HardwareMap map){
        DcMotor capstoneSlideDcMotor = map.dcMotor.get("capstoneSlideMotor");
        capstoneSlideDcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RobotMotorWithEncoder capstoneSlideMotor = new RobotMotorWithEncoder(capstoneSlideDcMotor,ElysiumSettings.CAPSTONE_SLIDE_MOTOR_TYPE);
        RobotMotorController capstoneSlideMotorController = new RobotMotorController(capstoneSlideMotor,true,2.0);
        this.capstoneSlide = new RobotServoUsingMotor(capstoneSlideMotorController,CAPSTONEDELIVERY_SLIDE_MIN_POS,CAPSTONEDELIVERY_SLIDE_MIN_POS,CAPSTONEDELIVERY_SLIDE_MAX_POS);
        this.capstoneRotServo = map.servo.get("capstoneRotServo");
        SensorUtil.setServoPulseWidth(this.capstoneRotServo,ElysiumSettings.CAPSTONE_SERVO_TYPE);
        this.setCapstoneServoOut(false);
    }

    public boolean isCapstoneServoOut(){
        return this.capstoneRotServo.getPosition() == ElysiumSettings.CAPSTONE_SERVO_OUT_POS;
    }

    public void setCapstoneServoOut(boolean out){
        if(out){
            this.capstoneRotServo.setPosition(ElysiumSettings.CAPSTONE_SERVO_OUT_POS);
        }else{
            this.capstoneRotServo.setPosition(ElysiumSettings.CAPSTONE_SERVO_IN_POS);
        }
    }

    @Override
    public boolean isBusy() {
        return this.capstoneSlide.isBusy();
    }

    @Override
    public void updateStatus() {
        this.capstoneSlide.updateStatus();
    }

    public void save(){
        FTCMemory.setSetting("ElysiumCapstoneSlidePos",capstoneSlide.getCurrentPosition());
    }

    public void read(){
        Double memorySlidePosition = null;
        try{
            memorySlidePosition = FTCMemory.getSetting("ElysiumCapstoneSlidePos",this.capstoneSlide.getCurrentPosition());
        }catch(Exception e){
            memorySlidePosition = this.capstoneSlide.getCurrentPosition();
        }
        this.capstoneSlide.adjustCurrentPosition(memorySlidePosition);
    }

    @Override
    public void waitUntilFinish() {
        while(this.isBusy()){
            this.updateStatus();
        }
    }

    public void stop(){
        this.capstoneSlide.deleteAllTasks();
    }

    public void updateTelemetry(Telemetry telemetry, TelemetryPacket telemetryPacket){
        GlobalUtil.addTelmetryLine(telemetry,telemetryPacket,"capstoneDeliverySystemSlide","" + capstoneSlide.getCurrentPosition());
        GlobalUtil.addTelmetryLine(telemetry,telemetryPacket,"capstoneDeliveryServo","" + (this.isCapstoneServoOut() ? "Out" : "In"));
    }
}
