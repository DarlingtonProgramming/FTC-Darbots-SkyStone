package org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.darbots.darbotsftclib.libcore.OpModes.DarbotsBasicOpMode;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPose2D;
import org.darbots.darbotsftclib.libcore.tasks.chassis_tasks.RobotMotionSystemTeleOpTask;
import org.darbots.darbotsftclib.libcore.templates.DarbotsAction;
import org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.ElysiumCore;
import org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.ElysiumSoundBox;
import org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.Elysium_Settings.ElysiumSettings;
import org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.Elysium_Settings.ElysiumTeleOpSettings;
import org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.Soundboxes.ElysiumTeleOpSoundBox;
import org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.Subsystems.ElysiumIntake;
import org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.Subsystems.ElysiumOuttake;

@TeleOp(name = "Elysium-TeleOp", group = "4100")
public class ElysiumTeleOp extends DarbotsBasicOpMode<ElysiumCore> {
    public class AutoCalibrateAction extends DarbotsAction{
        int step = 0;
        ElapsedTime intakeTimeCounter = null;
        @Override
        protected void __startAction() {
            step = 0;
            getRobotCore().outtakeSubSystem.outTakeSlide.setTargetPosition(
                    getRobotCore().outtakeSubSystem.OUTTAKE_OUT_OF_WAY_INTAKE_POS,
                    1.0
            );
        }

        @Override
        protected void __stopAction() {
            step = 0;
            intakeTimeCounter = null;
        }

        @Override
        public void updateStatus() {
            if(!this.isBusy()){
                return;
            }
            switch(step){
                case 0:
                    if(!getRobotCore().outtakeSubSystem.outTakeSlide.isBusy()){
                        step = 1;
                        getRobotCore().intakeSubSystem.setIntakeSystemStatus(ElysiumIntake.ElysiumIntakeStatus.SUCKING,1.0);
                        intakeTimeCounter = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
                    }
                case 1:
                    if(intakeTimeCounter.seconds() >= 1){
                        step = 2;
                        intakeTimeCounter = null;
                        getRobotCore().outtakeSubSystem.outTakeSlide.setTargetPosition(getRobotCore().outtakeSubSystem.OUTTAKE_SLIDE_MIN_POS,1.0);
                    }
                case 2:
                    if(!getRobotCore().outtakeSubSystem.outTakeSlide.isBusy()){
                        step = 3;
                    }
                default:
                    this.stopAction();
                    return;
            }
        }
    }
    public class OutTakeContorlAction extends DarbotsAction{
        int step = 0;
        ElapsedTime commonTimer = null;

        @Override
        protected void __startAction() {
            step = 0;
            commonTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
            getRobotCore().outtakeSubSystem.outTakeSlide.setTargetPosition(getRobotCore().outtakeSubSystem.OUTTAKE_SLIDE_MAX_POS,ElysiumTeleOpSettings.OUTTAKE_SLIDE_SPEED);
            getRobotCore().outtakeSubSystem.setGrabberState(ElysiumOuttake.Outtake_Grabber_State.GRABBED);
        }

        @Override
        protected void __stopAction() {
            step = 0;
            commonTimer = null;
            getRobotCore().outtakeSubSystem.outTakeSlide.setTargetPosition(getRobotCore().outtakeSubSystem.OUTTAKE_SLIDE_MIN_POS,ElysiumTeleOpSettings.OUTTAKE_SLIDE_SPEED);
            getRobotCore().outtakeSubSystem.setGrabberState(ElysiumOuttake.Outtake_Grabber_State.GRABBED);
        }

        @Override
        public void updateStatus() {
            if(!this.isBusy()){
                return;
            }
            switch(step){
                case 0:
                    if(!getRobotCore().outtakeSubSystem.outTakeSlide.isBusy()){
                        step = 1;
                        commonTimer.reset();
                        getRobotCore().outtakeSubSystem.setGrabberState(ElysiumOuttake.Outtake_Grabber_State.RELEASED);
                    }
                case 1:
                    if(commonTimer.seconds() >= ElysiumSettings.OUTTAKE_GRABBER_WAIT_SEC){
                        step = 2;
                        commonTimer.reset();
                        getRobotCore().outtakeSubSystem.outTakeSlide.setTargetPosition(getRobotCore().outtakeSubSystem.OUTTAKE_SLIDE_MIN_POS,ElysiumTeleOpSettings.OUTTAKE_SLIDE_SPEED);
                    }
                case 2:
                    if(!getRobotCore().outtakeSubSystem.outTakeSlide.isBusy()){
                        step = 3;
                    }
                default:
                    this.stopAction();
                    return;
            }
        }
    }
    public class IntakePositioningAction extends DarbotsAction{
        int step = 0;
        ElapsedTime commonTimer = null;
        @Override
        protected void __startAction() {
            commonTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
            step = 0;
            getRobotCore().intakeSubSystem.setPositioningServoStatus(ElysiumIntake.ElysiumIntakePositioningServoStatus.HIT);
            getRobotCore().outtakeSubSystem.setGrabberState(ElysiumOuttake.Outtake_Grabber_State.RELEASED);
        }

        @Override
        protected void __stopAction() {
            commonTimer = null;
            getRobotCore().intakeSubSystem.setPositioningServoStatus(ElysiumIntake.ElysiumIntakePositioningServoStatus.REST);
            step = 0;
        }

        @Override
        public void updateStatus() {
            if(!this.isBusy()){
                return;
            }
            switch(step){
                case 0:
                    if(commonTimer.seconds() >= ElysiumSettings.INTAKE_POSITIONING_SERVO_WAIT_SEC){
                        step = 1;
                        getRobotCore().outtakeSubSystem.setGrabberState(ElysiumOuttake.Outtake_Grabber_State.GRABBED);
                        commonTimer.reset();
                    }
                case 1:
                    if(commonTimer.seconds() >= ElysiumSettings.OUTTAKE_GRABBER_WAIT_SEC){
                        step = 2;
                        getRobotCore().intakeSubSystem.setPositioningServoStatus(ElysiumIntake.ElysiumIntakePositioningServoStatus.REST);
                        commonTimer.reset();
                    }
                case 2:
                    if(commonTimer.seconds() >= ElysiumSettings.INTAKE_POSITIONING_SERVO_WAIT_SEC){
                        step = 3;
                    }
                default:
                    this.stopAction();
                    return;
            }
        }
    }

    private ElysiumCore m_Core;
    private int telemetry_i = 0;
    private RobotMotionSystemTeleOpTask teleOpTask;
    public ElysiumTeleOpSoundBox SoundBox;
    private OutTakeContorlAction outtakeControlAction;
    private IntakePositioningAction intakePositioningAction;

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
        telemetry_i = 0;

        this.outtakeControlAction = new OutTakeContorlAction();
        this.intakePositioningAction = new IntakePositioningAction();
    }

    @Override
    public void hardwareDestroy() {
        this.SoundBox.terminate();
    }

    @Override
    public void RunThisOpMode() {
        this.SoundBox.onStart();
        this.teleopAutoMovement();
        while(this.opModeIsActive()){
            controlLoop();

            this.SoundBox.updateStatus();
            this.updateStatus();
            lazyTelemetry();
        }
    }

    public void teleopAutoMovement(){
        AutoCalibrateAction autoCalibrateAction = new AutoCalibrateAction();
        autoCalibrateAction.startAction();
        while(autoCalibrateAction.isBusy() && opModeIsActive()){
            driveControl();
            this.updateStatus();
            autoCalibrateAction.updateStatus();
            lazyTelemetry();
        }
    }

    public void controlLoop(){
        {
            //gamepad 1 zone
            this.driveControl();
            this.intakeControl();
            this.outtakeControl();
        }
        {
            //gamepad 2 zone
            
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
        if(gamepad1.x){
            if(this.outtakeControlAction.isBusy()){
                this.outtakeControlAction.stopAction();
            }
            this.intakePositioningAction.startAction();
        }
    }

    public void outtakeControl(){
        if(gamepad1.y) {
            if(this.intakePositioningAction.isBusy()){
                this.intakePositioningAction.stopAction();
            }
            this.outtakeControlAction.startAction();
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

    public void lazyTelemetry(){
        if(telemetry_i>=CONST_TELMETRY_PACKET_CYCLE_TIME) {
            telemetryCycle();
            telemetry_i = 0;
        }else{
            telemetry_i++;
        }
    }

    public void updateStatus(){
        this.m_Core.updateStatus();
    }
}
