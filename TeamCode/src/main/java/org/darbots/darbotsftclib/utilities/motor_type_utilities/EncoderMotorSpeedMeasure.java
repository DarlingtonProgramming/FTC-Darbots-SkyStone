package org.darbots.darbotsftclib.utilities.motor_type_utilities;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.darbots.darbotsftclib.libcore_4_5_0Pre.OpModes.DarbotsBasicOpMode;
import org.darbots.darbotsftclib.libcore_4_5_0Pre.motortypes.AndyMark3637;
import org.darbots.darbotsftclib.libcore_4_5_0Pre.sensors.motors.RobotMotorWithEncoder;
import org.darbots.darbotsftclib.libcore_4_5_0Pre.templates.RobotCore;
import org.darbots.darbotsftclib.libcore_4_5_0Pre.templates.motor_related.MotorType;
import org.darbots.darbotsftclib.libcore_4_5_0Pre.templates.motor_related.RobotMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(group = "DarbotsLib-Utilities", name = "EncoderMotorSpeedMeasureUtility")
public class EncoderMotorSpeedMeasure extends DarbotsBasicOpMode {
    public static final String motorConfigName = "motor";
    public static final MotorType motorCountsPerRevProvider = new AndyMark3637();
    public static final double motorTestDurationInSec = 10.0;
    public static final double motorTestAccelDurationInSec = 5.0;
    public static final double motorTestAccelAndCruiseWaitDuration = 1.0;

    private RobotMotorWithEncoder m_Motor;

    @Override
    public RobotCore getRobotCore() {
        return null;
    }

    @Override
    public void hardwareInitialize() {
        this.m_Motor = new RobotMotorWithEncoder(hardwareMap.dcMotor.get(motorConfigName),motorCountsPerRevProvider);
        this.m_Motor.setCurrentMovingType(RobotMotor.MovingType.reset);
        this.m_Motor.setCurrentMovingType(RobotMotor.MovingType.withSpeed);
    }

    @Override
    public void hardwareDestroy() {

    }

    @Override
    public void RunThisOpMode() {
        telemetry.addData("Info","Accelerating...");
        telemetry.update();
        ElapsedTime time = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        while(time.seconds() < motorTestAccelDurationInSec){
            double progress = Range.clip(time.seconds() / motorTestAccelDurationInSec, 0, 1.0);
            this.m_Motor.setPower(progress);
            if(isStopRequested()){
                return;
            }
        }
        telemetry.addData("Info","Waiting For RPM Counting to Start");
        telemetry.update();
        this.m_Motor.setPower(1.0);
        time.reset();
        while(time.seconds() < motorTestAccelAndCruiseWaitDuration){
            if(isStopRequested()){
                return;
            }
        }

        telemetry.addData("Info","Counting for RPM");
        telemetry.update();
        int startCount = this.m_Motor.getCurrentCount();
        time.reset();
        while(time.seconds() < motorTestDurationInSec){
            if(isStopRequested()){
                return;
            }
        }
        double countedDuration = time.seconds();
        int endCount = this.m_Motor.getCurrentCount();
        int deltaCount = endCount - startCount;
        double deltaCycle = deltaCount / this.motorCountsPerRevProvider.getCountsPerRev();
        double RevPerSec = deltaCycle / countedDuration;
        double RevPerMinute = RevPerSec * 60.0;

        telemetry.addData("Info","Decelerating..");
        Telemetry.Line RPMDataLine = telemetry.addLine();
        RPMDataLine.addData("RPS",RevPerSec);
        RPMDataLine.addData("RPM",RevPerMinute);
        telemetry.update();
        time.reset();
        while(time.seconds() < motorTestAccelDurationInSec){
            double progress = Range.clip(time.seconds() / motorTestAccelDurationInSec, 0, 1.0);
            this.m_Motor.setPower(1.0 - progress);
            if(isStopRequested()){
                return;
            }
        }

        RPMDataLine = telemetry.addLine();
        RPMDataLine.addData("RPS",RevPerSec);
        RPMDataLine.addData("RPM",RevPerMinute);
        telemetry.update();

        while(!isStopRequested()){
            sleep(50);
        }
    }
}
