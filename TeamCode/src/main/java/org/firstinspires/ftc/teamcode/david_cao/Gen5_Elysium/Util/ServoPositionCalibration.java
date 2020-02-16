package org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.Util;

import android.hardware.Sensor;

import com.qualcomm.robotcore.hardware.Servo;

import org.darbots.darbotsftclib.libcore.OpModes.DarbotsBasicOpMode;
import org.darbots.darbotsftclib.libcore.runtime.SensorUtil;
import org.darbots.darbotsftclib.libcore.templates.RobotCore;

public class ServoPositionCalibration extends DarbotsBasicOpMode {
    private Servo mServo;

    @Override
    public RobotCore getRobotCore() {
        return null;
    }

    @Override
    public void hardwareInitialize() {
        this.mServo = hardwareMap.servo.get(ServoPositionCalibrationSettings.servoPositionCalibrationConfigName);
        SensorUtil.setServoPulseWidth(this.mServo,ServoPositionCalibrationSettings.servoPositionCalibrationServoType);
    }

    @Override
    public void hardwareDestroy() {

    }

    @Override
    public void RunThisOpMode() {
        while(this.opModeIsActive()){
            this.mServo.setPosition(ServoPositionCalibrationSettings.servoPositionCalibrationPos);
        }
    }
}
