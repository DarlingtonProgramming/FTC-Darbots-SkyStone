package org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.Util;

import android.hardware.Sensor;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.darbots.darbotsftclib.libcore.OpModes.DarbotsBasicOpMode;
import org.darbots.darbotsftclib.libcore.runtime.SensorUtil;
import org.darbots.darbotsftclib.libcore.templates.RobotCore;

@TeleOp(name = "Elysium-Util-ServoCalibration",group = "4100")
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
        this.telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void hardwareDestroy() {

    }

    @Override
    public void RunThisOpMode() {
        while(this.opModeIsActive()){
            this.mServo.setPosition(ServoPositionCalibrationSettings.servoPositionCalibrationPos);
            this.updateTelmetry();
            telemetry.update();
        }
    }

    public void updateTelmetry(){
        telemetry.addData("ServoPosition",this.mServo.getPosition());
    }
}
