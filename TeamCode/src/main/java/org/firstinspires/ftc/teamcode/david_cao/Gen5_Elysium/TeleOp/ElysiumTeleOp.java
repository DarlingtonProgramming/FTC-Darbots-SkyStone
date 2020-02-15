package org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.darbots.darbotsftclib.libcore.OpModes.DarbotsBasicOpMode;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPose2D;
import org.darbots.darbotsftclib.libcore.tasks.chassis_tasks.RobotMotionSystemTeleOpTask;
import org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.ElysiumCore;
import org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.ElysiumSoundBox;
import org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.Elysium_Settings.ElysiumTeleOpSettings;
import org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.Soundboxes.ElysiumTeleOpSoundBox;
import org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.Subsystems.ElysiumIntake;

public class ElysiumTeleOp extends DarbotsBasicOpMode<ElysiumCore> {
    private ElysiumCore m_Core;
    private int telemetry_i = 0;
    private RobotMotionSystemTeleOpTask teleOpTask;
    private ElysiumTeleOpSoundBox SoundBox;
    @Override
    public ElysiumCore getRobotCore() {
        return this.m_Core;
    }

    @Override
    public void hardwareInitialize() {
        this.m_Core = new ElysiumCore("ElysiumTeleOp.log",this.hardwareMap,true,new RobotPose2D(0,0,0),false);
        this.teleOpTask = new RobotMotionSystemTeleOpTask();
        this.m_Core.getChassis().addTask(this.teleOpTask);
        this.SoundBox = new ElysiumTeleOpSoundBox(this);
        this.SoundBox.onInitialize();
    }

    @Override
    public void hardwareDestroy() {
        this.SoundBox.terminate();
    }

    @Override
    public void RunThisOpMode() {
        this.SoundBox.onStart();
        this.telemetry_i = 0;
        while(this.opModeIsActive()){
            controlLoop();
        }
    }

    public void controlLoop(){
        this.driveControl();
        this.intakeControl();

        this.updateStatus();
        this.SoundBox.updateStatus();

        if(telemetry_i>=CONST_TELMETRY_PACKET_CYCLE_TIME) {
            telemetryCycle();
            telemetry_i = 0;
        }else{
            telemetry_i++;
        }
    }

    public void driveControl(){
        double controlX = 0, controlY = 0, controlRotZ = 0;
        if(Math.abs(gamepad1.left_stick_y) >= ElysiumTeleOpSettings.GAMEPAD_THRESEHOLD) {
            controlX = -gamepad1.left_stick_y;
        }
        if(Math.abs(gamepad1.left_stick_x) >= ElysiumTeleOpSettings.GAMEPAD_THRESEHOLD) {
            controlY = -gamepad1.left_stick_x;
        }
        if(Math.abs(gamepad1.right_stick_x) >= ElysiumTeleOpSettings.GAMEPAD_THRESEHOLD) {
            controlRotZ = -gamepad1.right_stick_x;
        }

        this.teleOpTask.xSpeedNormalized = controlX * ElysiumTeleOpSettings.CHASSIS_SPEED_X_FACTOR;
        this.teleOpTask.ySpeedNormalized = controlY * ElysiumTeleOpSettings.CHASSIS_SPEED_Y_FACTOR;
        this.teleOpTask.zRotSpeedNormalized = controlRotZ * ElysiumTeleOpSettings.CHASSIS_SPEED_ROT_FACTOR;
    }

    public void intakeControl(){
        if(this.gamepad1.right_trigger >= ElysiumTeleOpSettings.GAMEPAD_THRESEHOLD){
            this.m_Core.intakeSubSystem.setIntakeSystemStatus(ElysiumIntake.ElysiumIntakeStatus.SUCKING, ElysiumTeleOpSettings.INTAKE_MAX_SPEED);
        }else if(this.gamepad1.left_trigger >= ElysiumTeleOpSettings.GAMEPAD_THRESEHOLD){
            this.m_Core.intakeSubSystem.setIntakeSystemStatus(ElysiumIntake.ElysiumIntakeStatus.SPITTING, ElysiumTeleOpSettings.INTAKE_MAX_SPEED);
        }else{
            this.m_Core.intakeSubSystem.setIntakeSystemStatus(ElysiumIntake.ElysiumIntakeStatus.STOPPED, 0);
        }
    }

    public TelemetryPacket updateTelemetry(){
        TelemetryPacket packet = this.m_Core.updateTelemetry();
        return packet;
    }

    public void telemetryCycle(){
        TelemetryPacket packet = this.updateTelemetry();
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
        telemetry.update();
    }

    public void updateStatus(){
        this.m_Core.updateStatus();
    }
}
