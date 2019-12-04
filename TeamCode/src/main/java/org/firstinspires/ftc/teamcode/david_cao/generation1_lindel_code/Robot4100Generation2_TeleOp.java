package org.firstinspires.ftc.teamcode.david_cao.generation1_lindel_code;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.darbots.darbotsftclib.libcore.OpModes.DarbotsBasicOpMode;
import org.darbots.darbotsftclib.libcore.integratedfunctions.LoopableTimer;
import org.darbots.darbotsftclib.libcore.tasks.servo_tasks.motor_powered_servo_tasks.TargetPosTask;
import org.darbots.darbotsftclib.libcore.templates.chassis_related.RobotMotionSystemTeleOpControlTask;

@TeleOp(name = "4100Gen1TeleOp_dcao",group = "4100")
public class Robot4100Generation2_TeleOp extends DarbotsBasicOpMode<Robot4100Generation2_LindelCore> {
    private Robot4100Generation2_LindelCore m_RobotCore;
    private LoopableTimer m_TimerStoneOrient = null;
    private LoopableTimer m_TimerCapStoneDelivery = null;
    private double m_SpeedFactor = 1.0;
    private boolean keepSucking = true;

    @Override
    public Robot4100Generation2_LindelCore getRobotCore() {
        return m_RobotCore;
    }

    @Override
    public void hardwareInitialize() {
        this.m_RobotCore = new Robot4100Generation2_LindelCore(this.hardwareMap);
        this.m_RobotCore.getChassis().replaceTask(this.m_RobotCore.getChassis().getTeleOpTask());
        this.m_RobotCore.getLinearSlide().adjustCurrentPosition(Robot4100Generation2_Settings.LINEARSLIDE_GRAB);
    }

    @Override
    public void hardwareDestroy() {
        this.m_RobotCore.terminate();
    }

    @Override
    public void RunThisOpMode() {
        waitForStart();
        while(this.opModeIsActive()){

            //Chassis Control
            if(gamepad1.x){
                this.m_SpeedFactor = 0.5;
            }else if(gamepad1.y){
                this.m_SpeedFactor = 1.0;
            }
            RobotMotionSystemTeleOpControlTask teleOpTask = (RobotMotionSystemTeleOpControlTask) this.m_RobotCore.getChassis().getCurrentTask();
            double ZAxis = -gamepad1.left_stick_y, XAxis = gamepad1.left_stick_x, Turn = (-gamepad1.right_stick_x);//gamepad1.left_trigger - gamepad1.right_trigger;
            if(Math.abs(ZAxis) < 0.1){
                ZAxis = 0;
            }
            if(Math.abs(XAxis) < 0.1){
                XAxis = 0;
            }
            if(Math.abs(Turn) < 0.1){
                Turn = 0;
            }
            ZAxis *= Robot4100Generation2_Settings.TELEOP_MAXSPEED;
            XAxis *= Robot4100Generation2_Settings.TELEOP_MAXSPEED;
            Turn *= Robot4100Generation2_Settings.TELEOP_MAXSPEED;
            double SlowDownFactor = this.m_SpeedFactor;
            ZAxis *= SlowDownFactor;
            XAxis *= SlowDownFactor;
            Turn *= SlowDownFactor;
            teleOpTask.setDriveXSpeed(XAxis);
            teleOpTask.setDriveZSpeed(ZAxis);
            teleOpTask.setDriveRotationSpeed(Turn);

            if(gamepad2.left_stick_y < -0.1){
                if((!this.m_RobotCore.getLinearSlide().isBusy())) {
                    this.m_RobotCore.getLinearSlide().replaceTask(new TargetPosTask(null, this.m_RobotCore.getLinearSlide().getMaxPos(), Robot4100Generation2_Settings.TELEOP_LINEARSLIDESPEED));
                }
            }else if(gamepad2.left_stick_y > 0.1){
                if((!this.m_RobotCore.getLinearSlide().isBusy())) {
                    this.m_RobotCore.getLinearSlide().replaceTask(new TargetPosTask(null, this.m_RobotCore.getLinearSlide().getMinPos(), Robot4100Generation2_Settings.TELEOP_LINEARSLIDESPEED));
                }
            }else{
                this.m_RobotCore.getLinearSlide().deleteAllTasks();
            }

            //Gamepad1 Intake System Control
            if(gamepad1.right_trigger > 0.2){//gamepad1.b){
                this.m_RobotCore.setIntakeSystemStatus(Robot4100Generation2_LindelCore.IntakeSystemStatus.SUCK,gamepad1.right_trigger * Robot4100Generation2_Settings.INTAKEMOTOR_SPEED);
                keepSucking = true;
            }else if(gamepad1.left_trigger > 0.2){ //gamepad1.y){
                this.m_RobotCore.setIntakeSystemStatus(Robot4100Generation2_LindelCore.IntakeSystemStatus.VOMIT, gamepad1.left_trigger * Robot4100Generation2_Settings.INTAKEMOTOR_SPEED);
                keepSucking = false;
            }else{
                if(keepSucking){
                    this.m_RobotCore.setIntakeSystemStatus(Robot4100Generation2_LindelCore.IntakeSystemStatus.SUCK, Robot4100Generation2_Settings.INTAKEMOTOR_SPEED);
                }else {
                    this.m_RobotCore.setIntakeSystemStatus(Robot4100Generation2_LindelCore.IntakeSystemStatus.STOP, 0);
                }
            }

            if(gamepad2.right_bumper){
                this.m_RobotCore.setGrabberServoToGrab(false);
            }else{
                this.m_RobotCore.setGrabberServoToGrab(true);
            }
            if(gamepad2.right_trigger > 0){
                this.m_RobotCore.setGrabberRotServoToOutside(true, Robot4100Generation2_Settings.TELEOP_GRABBERROT_OUT_SPEED);
            }else if(gamepad2.left_trigger > 0){
                this.m_RobotCore.setGrabberRotServoToOutside(false, Robot4100Generation2_Settings.TELEOP_GRABBERROT_IN_SPEED);
            }

            if(gamepad2.a){
                //Set StoneOrientServo to Orient Pos, then come back
                if(this.m_TimerStoneOrient == null) {
                    this.m_RobotCore.setOrientServoToOrient(true);
                    this.m_TimerStoneOrient = new LoopableTimer(0.8) {
                        @Override
                        protected void run() {
                            m_RobotCore.setOrientServoToOrient(false);
                            m_TimerStoneOrient = null;
                        }
                    };
                    this.m_TimerStoneOrient.start();
                }
            }
            if(gamepad2.x){
                if(m_TimerCapStoneDelivery == null){
                    this.m_RobotCore.setCapStoneServoToDeposit(true);
                    this.m_TimerCapStoneDelivery = new LoopableTimer(1.0) {
                        @Override
                        protected void run() {
                            m_RobotCore.setCapStoneServoToDeposit(false);
                            m_TimerCapStoneDelivery = null;
                        }
                    };
                    this.m_TimerCapStoneDelivery.start();
                }
            }

            if(gamepad2.dpad_down){
                this.m_RobotCore.setDragServoToDrag(true);
            }else if(gamepad2.dpad_up){
                this.m_RobotCore.setDragServoToDrag(false);
            }

            this.m_RobotCore.updateStatus();
            if(this.m_TimerStoneOrient != null){
                this.m_TimerStoneOrient.updateStatus();
            }
            if(this.m_TimerCapStoneDelivery != null){
                this.m_TimerCapStoneDelivery.updateStatus();
            }
            telemetry.addData("LinearSlide","" + this.m_RobotCore.getLinearSlide().getCurrentPosition() + "[" + this.m_RobotCore.getLinearSlide().getCurrentPercent() + "%]," + (this.m_RobotCore.getLinearSlide().isBusy() ? "Busy" : "Not Busy"));
            telemetry.update();
        }
    }
}
