package org.darbots.darbotsftclib.utilities.motor_type_utilities;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.darbots.darbotsftclib.libcore.OpModes.DarbotsBasicOpMode;
import org.darbots.darbotsftclib.libcore.motortypes.GoBilda5202Series1150RPMMotor;
import org.darbots.darbotsftclib.libcore.runtime.GlobalUtil;
import org.darbots.darbotsftclib.libcore.sensors.motors.RobotMotorWithEncoder;
import org.darbots.darbotsftclib.libcore.templates.RobotCore;
import org.darbots.darbotsftclib.libcore.templates.motor_related.MotorType;
import org.darbots.darbotsftclib.libcore.templates.motor_related.RobotMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(group = "DarbotsLib-Utilities", name = "EncoderCountsPerRevMeasureUtility")
public class EncoderCountsPerRevMeasure extends DarbotsBasicOpMode {
    public static final String motorConfigName = "RF";
    public static final MotorType motorCountsPerRevProvider = new GoBilda5202Series1150RPMMotor();

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
            TelemetryPacket packet = this.updateTelemetry();
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
            telemetry.update();
        }
    }

    public TelemetryPacket updateTelemetry(){
        TelemetryPacket packet = new TelemetryPacket();
        GlobalUtil.addTelmetryLine(this.telemetry,packet,"Current Revolution","" + m_Motor.getCurrentCount() / motorCountsPerRevProvider.getCountsPerRev());
        GlobalUtil.addTelmetryLine(this.telemetry,packet,"CPR","" + motorCountsPerRevProvider.getCountsPerRev());
        GlobalUtil.addTelmetryLine(this.telemetry,packet,"Info","Please rotate the motor manually and read the number of revolutions it went through to tune it.");
        return packet;
    }
}
