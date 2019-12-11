package org.darbots.darbotsftclib.utilities.motor_type_utilities;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.darbots.darbotsftclib.libcore.OpModes.DarbotsBasicOpMode;
import org.darbots.darbotsftclib.libcore.motortypes.AndyMark3637;
import org.darbots.darbotsftclib.libcore.sensors.motors.RobotMotorWithEncoder;
import org.darbots.darbotsftclib.libcore.templates.RobotCore;
import org.darbots.darbotsftclib.libcore.templates.motor_related.MotorType;
import org.darbots.darbotsftclib.libcore.templates.motor_related.RobotMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(group = "DarbotsLib-Utilities", name = "EncoderCountsPerRevMeasureUtility")
public class EncoderCountsPerRevMeasure extends DarbotsBasicOpMode {
    public static final String motorConfigName = "motor";
    public static final MotorType motorCountsPerRevProvider = new AndyMark3637();

    private RobotMotorWithEncoder m_Motor;

    @Override
    public RobotCore getRobotCore() {
        return null;
    }

    @Override
    public void hardwareInitialize() {
        this.m_Motor = new RobotMotorWithEncoder(hardwareMap.dcMotor.get(motorConfigName),motorCountsPerRevProvider);
        this.m_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.m_Motor.setCurrentMovingType(RobotMotor.MovingType.reset);
        this.m_Motor.setCurrentMovingType(RobotMotor.MovingType.withSpeed);

    }

    @Override
    public void hardwareDestroy() {

    }

    @Override
    public void RunThisOpMode() {
        while(this.opModeIsActive()){
            this.updateTelemetry();
        }
    }

    public void updateTelemetry(){
        telemetry.addData("Current Revolution",m_Motor.getCurrentCount() / motorCountsPerRevProvider.getCountsPerRev());
        telemetry.addData("CPR",motorCountsPerRevProvider.getCountsPerRev());
        telemetry.addData("Info","Please rotate the motor manually and read the number of revolutions it went through to tune it.");
        telemetry.update();
    }
}
