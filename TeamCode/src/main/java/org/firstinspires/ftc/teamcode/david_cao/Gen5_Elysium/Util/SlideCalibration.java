package org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.Util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.darbots.darbotsftclib.libcore.OpModes.DarbotsBasicOpMode;
import org.darbots.darbotsftclib.libcore.sensors.motors.RobotMotorController;
import org.darbots.darbotsftclib.libcore.sensors.motors.RobotMotorWithEncoder;
import org.darbots.darbotsftclib.libcore.sensors.servos.motor_powered_servos.RobotServoUsingMotor;
import org.darbots.darbotsftclib.libcore.tasks.servo_tasks.motor_powered_servo_tasks.TargetPosTask;
import org.darbots.darbotsftclib.libcore.templates.RobotCore;

public class SlideCalibration extends DarbotsBasicOpMode {
    private RobotServoUsingMotor servoUsingMotor;

    @Override
    public RobotCore getRobotCore() {
        return null;
    }

    @Override
    public void hardwareInitialize() {
        DcMotor motor = hardwareMap.dcMotor.get(SlideCalibrationSettings.slideCalibrationMotorConfigName);
        if(SlideCalibrationSettings.slideCalibrationReversed){
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        RobotMotorController motorController = new RobotMotorController(new RobotMotorWithEncoder(motor,SlideCalibrationSettings.slideCalibrationMotorType),false,1.0);
        this.servoUsingMotor = new RobotServoUsingMotor(motorController,0,SlideCalibrationSettings.slideCalibrationMinPos,SlideCalibrationSettings.slideCalibrationMaxPos);
        this.telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void hardwareDestroy() {

    }

    @Override
    public void RunThisOpMode() {
        while(this.opModeIsActive()){
            if(gamepad1.left_stick_y < 0.15){
                if(!this.servoUsingMotor.isBusy()){
                    this.servoUsingMotor.replaceTask(new TargetPosTask(null,this.servoUsingMotor.getMaxPos(),SlideCalibrationSettings.slideCalibrationSpeed));
                }
            }else if(gamepad1.left_stick_y > 0.15){
                if(!this.servoUsingMotor.isBusy()){
                    this.servoUsingMotor.replaceTask(new TargetPosTask(null,this.servoUsingMotor.getMinPos(),SlideCalibrationSettings.slideCalibrationSpeed));
                }
            }else{
                this.servoUsingMotor.deleteAllTasks();
            }
            this.updateTelmetry();
            telemetry.update();
        }
    }

    public void updateTelmetry(){
        this.telemetry.addData("Slide Position","" + this.servoUsingMotor.getCurrentPosition() + "[" + (this.servoUsingMotor.getCurrentPosition() / (this.servoUsingMotor.getMaxPos() - this.servoUsingMotor.getMinPos())) + "%]");
    }
}
