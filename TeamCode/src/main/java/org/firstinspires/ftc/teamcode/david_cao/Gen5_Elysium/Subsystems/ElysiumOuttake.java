package org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.Subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.darbots.darbotsftclib.libcore.integratedfunctions.FTCMemory;
import org.darbots.darbotsftclib.libcore.runtime.GlobalUtil;
import org.darbots.darbotsftclib.libcore.runtime.SensorUtil;
import org.darbots.darbotsftclib.libcore.sensors.servos.TimeControlledServo;
import org.darbots.darbotsftclib.libcore.templates.RobotNonBlockingDevice;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.Elysium_Settings.ElysiumSettings;

public class ElysiumOuttake implements RobotNonBlockingDevice {
    public static enum Outtake_Grabber_State{
        RELEASED,
        GRABBED
    }
    public TimeControlledServo outTakeSlide;
    private Servo grabber;

    private double lastSlidePosition;
    public ElysiumOuttake(HardwareMap map){
        Servo outTakeSlideServo = map.servo.get("outTakeSlideServo");
        SensorUtil.setServoPulseWidth(outTakeSlideServo,ElysiumSettings.OUTTAKE_SLIDE_SERVO_TYPE);
        this.outTakeSlide = new TimeControlledServo(outTakeSlideServo,ElysiumSettings.OUTTAKE_SLIDE_SERVO_TYPE,ElysiumSettings.OUTTAKE_SLIDE_MIN_POS,true);
        this.grabber = map.servo.get("outTakeGrabberServo");
        SensorUtil.setServoPulseWidth(this.grabber,ElysiumSettings.OUTTAKE_GRABBER_SERVO_TYPE);
        this.grabber.setPosition(ElysiumSettings.OUTTAKE_GRABBER_FAR_RELEASED_POS);
        this.lastSlidePosition = this.outTakeSlide.getCurrentPosition();
    }

    @Override
    public boolean isBusy() {
        return outTakeSlide.isBusy();
    }

    @Override
    public void updateStatus() {
        this.outTakeSlide.updateStatus();
        {
            double currentSlidePosition = this.outTakeSlide.getCurrentPosition();
            if((lastSlidePosition < ElysiumSettings.OUTTAKE_SLIDE_SAFE_FAR_RELEASE && currentSlidePosition >= ElysiumSettings.OUTTAKE_SLIDE_SAFE_FAR_RELEASE) || (lastSlidePosition >= ElysiumSettings.OUTTAKE_SLIDE_SAFE_FAR_RELEASE && currentSlidePosition < ElysiumSettings.OUTTAKE_SLIDE_SAFE_FAR_RELEASE)){
                if(this.getGrabberState() == Outtake_Grabber_State.RELEASED){
                    this.setGrabberState(Outtake_Grabber_State.RELEASED);
                }else{
                    lastSlidePosition = currentSlidePosition;
                }
            }else{
                lastSlidePosition = currentSlidePosition;
            }
        }
    }

    public Outtake_Grabber_State getGrabberState(){
        if(this.grabber.getPosition() == ElysiumSettings.OUTTAKE_GRABBER_GRABBED_POS){
            return Outtake_Grabber_State.GRABBED;
        }else{
            return Outtake_Grabber_State.RELEASED;
        }
    }

    public void setGrabberState(Outtake_Grabber_State grabberState){
        if(grabberState == Outtake_Grabber_State.GRABBED){
            this.grabber.setPosition(ElysiumSettings.OUTTAKE_GRABBER_GRABBED_POS);
        }else{
            double currentSlidePosition = this.outTakeSlide.getServo().getPosition();
            if(currentSlidePosition >= ElysiumSettings.OUTTAKE_SLIDE_SAFE_FAR_RELEASE){
                this.grabber.setPosition(ElysiumSettings.OUTTAKE_GRABBER_FAR_RELEASED_POS);
            }else{
                this.grabber.setPosition(ElysiumSettings.OUTTAKE_GRABBER_RELEASED_POS);
            }
            this.lastSlidePosition = currentSlidePosition;
        }
    }

    public void save(){
        FTCMemory.setSetting("ElysiumOuttakeSlidePos",outTakeSlide.getCurrentPosition());
    }

    public void read(){
        Double memorySlidePosition = null;
        try{
            memorySlidePosition = FTCMemory.getSetting("ElysiumOuttakeSlidePos",this.outTakeSlide.getCurrentPosition());
        }catch(Exception e){
            memorySlidePosition = this.outTakeSlide.getCurrentPosition();
        }
        this.outTakeSlide.adjustLastPosition(memorySlidePosition);
        this.outTakeSlide.getServo().setPosition(memorySlidePosition);
        this.lastSlidePosition = memorySlidePosition;
        this.setGrabberState(this.getGrabberState());
    }

    @Override
    public void waitUntilFinish() {
        while(this.isBusy()){
            this.updateStatus();
        }
    }

    public void stop(){
        this.outTakeSlide.stop();
    }

    public void updateTelemetry(Telemetry telemetry, TelemetryPacket telemetryPacket){
        double currentOuttakePos = outTakeSlide.getCurrentPosition();
        GlobalUtil.addTelmetryLine(telemetry,telemetryPacket,"OuttakeSlide","" + currentOuttakePos + "[" + (currentOuttakePos / (ElysiumSettings.OUTTAKE_SLIDE_MIN_POS - ElysiumSettings.OUTTAKE_SLIDE_MAX_POS)) + "%]");
        GlobalUtil.addTelmetryLine(telemetry,telemetryPacket,"OuttakeGrabber",this.getGrabberState().name());
    }
}
