package org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.Subsystems;

import com.qualcomm.robotcore.hardware.Servo;

import org.darbots.darbotsftclib.libcore.runtime.SensorUtil;
import org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.ElysiumSettings;

public class ElysiumAutoArm{
    public static enum GrabberServoState{
        WIDE_OPEN,
        CLOSED,
        GRAB_STONE
    }
    public static enum ArmRotServoState{
        OUT,
        IN,
        REST
    }
    private Servo ArmGrabber;
    private Servo ArmRot;
    private double armGrabber_CLOSED, armGrabber_OPEN, armGrabber_GRAB;
    private double armRot_IN, armRot_OUT, armRot_REST;
    public ElysiumAutoArm(Servo Grabber, Servo armRot, double GRABBER_CLOSED, double GRABBER_OPEN, double GRABBER_GRAB, double ROT_IN, double ROT_OUT, double ROT_REST){
        this.ArmGrabber = Grabber;
        SensorUtil.setServoPulseWidth(this.ArmGrabber, ElysiumSettings.AUTONOMOUS_CLAW_GRABSERVO_TYPE);
        this.ArmRot = armRot;
        SensorUtil.setServoPulseWidth(this.ArmRot,ElysiumSettings.AUTONOMOUS_CLAW_ROTSERVO_TYPE);
        this.armGrabber_CLOSED = GRABBER_CLOSED;
        this.armGrabber_OPEN = GRABBER_OPEN;
        this.armGrabber_GRAB = GRABBER_GRAB;
        this.armRot_IN = ROT_IN;
        this.armRot_OUT = ROT_OUT;
        this.armRot_REST = ROT_REST;
        this.setArmRotServoState(ArmRotServoState.REST);
        this.setGrabberServoState(GrabberServoState.CLOSED);
    }
    public GrabberServoState getGrabberServoState(){
        double grabberPosition = this.ArmGrabber.getPosition();
        if(grabberPosition == armGrabber_CLOSED){
            return GrabberServoState.CLOSED;
        }else if(grabberPosition == armGrabber_OPEN){
            return GrabberServoState.WIDE_OPEN;
        }else{
            return GrabberServoState.GRAB_STONE;
        }
    }
    public void setGrabberServoState(GrabberServoState state){
        if(state == GrabberServoState.CLOSED){
            this.ArmGrabber.setPosition(armGrabber_CLOSED);
        }else if(state == GrabberServoState.WIDE_OPEN){
            this.ArmGrabber.setPosition(armGrabber_OPEN);
        }else{
            this.ArmGrabber.setPosition(armGrabber_GRAB);
        }
    }
    public ArmRotServoState getArmRotServoState(){
        double armRotPosition = this.ArmRot.getPosition();
        if(armRotPosition == armRot_IN){
            return ArmRotServoState.IN;
        }else if(armRotPosition == armRot_OUT){
            return ArmRotServoState.OUT;
        }else{
            return ArmRotServoState.REST;
        }
    }
    public void setArmRotServoState(ArmRotServoState state){
        if(state == ArmRotServoState.IN){
            this.ArmRot.setPosition(armRot_IN);
        }else if(state == ArmRotServoState.OUT){
            this.ArmRot.setPosition(armRot_OUT);
        }else{
            this.ArmRot.setPosition(armRot_REST);
        }
    }
}
