package org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.Subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

import static org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.Elysium_Settings.ElysiumSettings.STACKER_SLIDE_MAX_POS;

public class ElysiumStacker implements RobotNonBlockingDevice {
    public static enum Stacker_Door_State{
        RELEASED,
        CLOSED
    }
    public RobotServoUsingMotor stackerSlide;
    private Servo leftDoor;
    private Servo rightDoor;

    public ElysiumStacker(HardwareMap map){
        DcMotor stackerSlideDcMotor = map.dcMotor.get("stackerSlideMotor");
        stackerSlideDcMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        stackerSlideDcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RobotMotorWithEncoder stackerSlideMotor = new RobotMotorWithEncoder(stackerSlideDcMotor,ElysiumSettings.STACKER_SLIDE_TYPE);
        RobotMotorController stackerSlideMotorController = new RobotMotorController(stackerSlideMotor,true,2.0);
        this.stackerSlide = new RobotServoUsingMotor(stackerSlideMotorController,0,ElysiumSettings.STACKER_SLIDE_MIN_POS,STACKER_SLIDE_MAX_POS);
        this.leftDoor = map.servo.get("stackerDoorLeftServo");
        SensorUtil.setServoPulseWidth(this.leftDoor,ElysiumSettings.STACKER_DOOR_SERVO_TYPE);
        this.rightDoor = map.servo.get("stackerDoorRightServo");
        SensorUtil.setServoPulseWidth(this.rightDoor,ElysiumSettings.STACKER_DOOR_SERVO_TYPE);
    }

    public void initializePosition(){
        this.setDoorState(Stacker_Door_State.RELEASED);
    }

    public Stacker_Door_State getDoorState(){
        if(this.leftDoor.getPosition() == ElysiumSettings.STACKER_DOOR_LEFT_CLOSED_POS){
            return Stacker_Door_State.CLOSED;
        }else{
            return Stacker_Door_State.RELEASED;
        }
    }

    public void setDoorState(Stacker_Door_State doorState){
        if(doorState == Stacker_Door_State.CLOSED){
            this.leftDoor.setPosition(ElysiumSettings.STACKER_DOOR_LEFT_CLOSED_POS);
            this.rightDoor.setPosition(ElysiumSettings.STACKER_DOOR_RIGHT_CLOSED_POS);
        }else{
            this.leftDoor.setPosition(ElysiumSettings.STACKER_DOOR_LEFT_OPEN_POS);
            this.rightDoor.setPosition(ElysiumSettings.STACKER_DOOR_RIGHT_OPEN_POS);
        }
    }

    public void save(){
        FTCMemory.setSetting("ElysiumStackerSlidePos",stackerSlide.getCurrentPosition());
    }

    public void read(){
        Double memorySlidePosition = null;
        try{
            memorySlidePosition = FTCMemory.getSetting("ElysiumStackerSlidePos",this.stackerSlide.getCurrentPosition());
        }catch(Exception e){
            memorySlidePosition = this.stackerSlide.getCurrentPosition();
        }
        this.stackerSlide.adjustCurrentPosition(memorySlidePosition);
    }

    @Override
    public boolean isBusy() {
        return this.stackerSlide.isBusy();
    }

    @Override
    public void updateStatus() {
        this.stackerSlide.updateStatus();
    }

    @Override
    public void waitUntilFinish() {
        while(this.isBusy()){
            this.stackerSlide.updateStatus();
        }
    }

    public void stop(){
        this.stackerSlide.deleteAllTasks();
    }

    public void updateTelemetry(Telemetry telemetry, TelemetryPacket telemetryPacket){
        GlobalUtil.addTelmetryLine(telemetry,telemetryPacket,"stackerSlide","" + this.stackerSlide.getCurrentPosition());
        GlobalUtil.addTelmetryLine(telemetry,telemetryPacket,"stackerDoor",this.getDoorState().name());
    }
}
