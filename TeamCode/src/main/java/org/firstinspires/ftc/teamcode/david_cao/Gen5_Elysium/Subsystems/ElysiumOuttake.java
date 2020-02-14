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
import org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.ElysiumSettings;

public class ElysiumOuttake implements RobotNonBlockingDevice {
    public static enum Outtake_Grabber_State{
        RELEASED,
        GRABBED
    }
    public static final double OUTTAKE_SLIDE_MAX_POS = ElysiumSettings.OUTTAKE_SLIDE_MAX_POS;
    public static final double OUTTAKE_SLIDE_MIN_POS = ElysiumSettings.OUTTAKE_SLIDE_MIN_POS;
    public static final double OUTTAKE_OUT_OF_WAY_INTAKE_POS = ElysiumSettings.OUTTAKE_SLIDE_OUT_OF_WAY_INTAKE_POS;
    private static final double OUTTAKE_GRABBER_RELEASED_POS = ElysiumSettings.OUTTAKE_GRABBER_RELEASED_POS;
    private static final double OUTTAKE_GRABBER_GRABBED_POS = ElysiumSettings.OUTTAKE_GRABBER_GRABBED_POS;
    private static final double OUTTAKE_GRABBER_FAR_RELEASED_POS = ElysiumSettings.OUTTAKE_GRABBER_FAR_RELEASED_POS;
    private static final double OUTTAKE_SLIDE_MIDDLE_BETWEEN_RELEASED_AND_FAR_RELEASED = (OUTTAKE_SLIDE_MIN_POS + OUTTAKE_SLIDE_MAX_POS) / 2.0;
    public TimeControlledServo outTakeSlide;
    private Servo grabber;

    private double lastSlidePosition;
    public ElysiumOuttake(HardwareMap map){
        Servo outTakeSlideServo = map.servo.get("outTakeSlideServo");
        SensorUtil.setServoPulseWidth(outTakeSlideServo,ElysiumSettings.OUTTAKE_SLIDE_SERVO_TYPE);
        this.outTakeSlide = new TimeControlledServo(outTakeSlideServo,ElysiumSettings.OUTTAKE_SLIDE_SERVO_TYPE,OUTTAKE_SLIDE_MIN_POS,true);
        this.grabber = map.servo.get("outTakeGrabberServo");
        SensorUtil.setServoPulseWidth(this.grabber,ElysiumSettings.OUTTAKE_GRABBER_SERVO_TYPE);
        this.grabber.setPosition(OUTTAKE_GRABBER_FAR_RELEASED_POS);
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
            if((lastSlidePosition < OUTTAKE_SLIDE_MIDDLE_BETWEEN_RELEASED_AND_FAR_RELEASED && currentSlidePosition >= OUTTAKE_SLIDE_MIDDLE_BETWEEN_RELEASED_AND_FAR_RELEASED) || (lastSlidePosition >= OUTTAKE_SLIDE_MIDDLE_BETWEEN_RELEASED_AND_FAR_RELEASED && currentSlidePosition < OUTTAKE_SLIDE_MIDDLE_BETWEEN_RELEASED_AND_FAR_RELEASED)){
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
        if(this.grabber.getPosition() == OUTTAKE_GRABBER_GRABBED_POS){
            return Outtake_Grabber_State.GRABBED;
        }else{
            return Outtake_Grabber_State.RELEASED;
        }
    }

    public void setGrabberState(Outtake_Grabber_State grabberState){
        if(grabberState == Outtake_Grabber_State.GRABBED){
            this.grabber.setPosition(OUTTAKE_GRABBER_GRABBED_POS);
        }else{
            double currentSlidePosition = this.outTakeSlide.getCurrentPosition();
            if(currentSlidePosition < OUTTAKE_SLIDE_MIDDLE_BETWEEN_RELEASED_AND_FAR_RELEASED){
                this.grabber.setPosition(OUTTAKE_GRABBER_FAR_RELEASED_POS);
            }else{
                this.grabber.setPosition(OUTTAKE_GRABBER_RELEASED_POS);
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
        GlobalUtil.addTelmetryLine(telemetry,telemetryPacket,"OuttakeSlide","" + currentOuttakePos + "[" + (currentOuttakePos / (OUTTAKE_SLIDE_MAX_POS - OUTTAKE_SLIDE_MIN_POS)) + "%]");
        GlobalUtil.addTelmetryLine(telemetry,telemetryPacket,"OuttakeGrabber",this.getGrabberState().name());
    }
}
