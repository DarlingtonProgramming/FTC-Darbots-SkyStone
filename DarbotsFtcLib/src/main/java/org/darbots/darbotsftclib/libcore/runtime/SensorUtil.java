package org.darbots.darbotsftclib.libcore.runtime;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.PWMOutput;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;

import org.darbots.darbotsftclib.libcore.templates.servo_related.ContinuousRotationServoType;
import org.darbots.darbotsftclib.libcore.templates.servo_related.ServoType;

public class SensorUtil {
    public static void setServoPulseWidth(Servo servo, ServoType servoType){
        if(servo instanceof PwmControl){
            PwmControl servoPWM = (PwmControl) servo;
            servoPWM.setPwmRange(
                    new PwmControl.PwmRange(
                        servoType.getPulseLowerInMicroSeconds(),
                        servoType.getPulseUpperInMicroSeconds()
                    )
            );
        }
    }
    public static void setCRServoPulseWidth(CRServo crServo, ContinuousRotationServoType CRServoType){
        if(crServo instanceof PwmControl){
            PwmControl CRServoPWM = (PwmControl) crServo;
            CRServoPWM.setPwmRange(
                    new PwmControl.PwmRange(
                            CRServoType.getPulseLowerInMicroSeconds(),
                            CRServoType.getPulseUpperInMicroSeconds()
                    )
            );
        }
    }
}
