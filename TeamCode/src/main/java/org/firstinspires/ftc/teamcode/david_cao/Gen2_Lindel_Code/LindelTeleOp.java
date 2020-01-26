package org.firstinspires.ftc.teamcode.david_cao.Gen2_Lindel_Code;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.darbots.darbotsftclib.libcore_4_5_0Pre.OpModes.DarbotsBasicOpMode;
import org.darbots.darbotsftclib.libcore_4_5_0Pre.tasks.chassis_tasks.RobotMotionSystemTeleOpTask;
import org.darbots.darbotsftclib.libcore_4_5_0Pre.tasks.servo_tasks.motor_powered_servo_tasks.TargetPosTask;
import org.darbots.darbotsftclib.libcore_4_5_0Pre.templates.DarbotsComboKey;
import org.darbots.darbotsftclib.libcore_4_5_0Pre.templates.servo_related.motor_powered_servos.RobotServoUsingMotorTask;
import org.firstinspires.ftc.teamcode.david_cao.Gen2_Lindel_Code.ComboKeys.UNIVERSAL_AutoElevatorCombo;
import org.firstinspires.ftc.teamcode.david_cao.Gen2_Lindel_Code.ComboKeys.UNIVERSAL_AutoExitCombo;
import org.firstinspires.ftc.teamcode.david_cao.Gen2_Lindel_Code.ComboKeys.UNIVERSAL_BringElevatorDownCombo;
import org.firstinspires.ftc.teamcode.david_cao.Gen2_Lindel_Code.ComboKeys.UNIVERSAL_CapstoneCombo;
import org.firstinspires.ftc.teamcode.david_cao.Gen2_Lindel_Code.ComboKeys.UNIVERSAL_StoneOrientCombo;

@TeleOp(group = "4100", name = "LindelTeleOp-Gen2.1")
public class LindelTeleOp extends DarbotsBasicOpMode<LindelCore> {
    private LindelCore m_Core;
    private double lastStonePosition = LindelSettings.CONTROL_COMBO_STONE_INITILAL_HEIGHT;
    public double SpeedFactor = 1.0;
    private boolean keepSucking = true;
    private boolean capstoneOn = false;
    private RobotMotionSystemTeleOpTask driveTask = null;
    private DarbotsComboKey stoneOrientCombo;
    private DarbotsComboKey capstoneCombo;
    private UNIVERSAL_AutoElevatorCombo elevatorUpCombo;
    private UNIVERSAL_BringElevatorDownCombo elevatorDownCombo;
    private UNIVERSAL_AutoExitCombo stackUpCombo;

    @Override
    public LindelCore getRobotCore() {
        return m_Core;
    }

    @Override
    public void hardwareInitialize() {
        this.m_Core = new LindelCore(this.hardwareMap,"LindelGen2.1-TeleOp.log");
        driveTask = new RobotMotionSystemTeleOpTask();
        this.m_Core.getChassis().replaceTask(driveTask);
        stoneOrientCombo = new UNIVERSAL_StoneOrientCombo(this.m_Core);
        capstoneCombo = new UNIVERSAL_CapstoneCombo(this.m_Core);
        lastStonePosition = LindelSettings.CONTROL_COMBO_STONE_INITILAL_HEIGHT;
        elevatorUpCombo = new UNIVERSAL_AutoElevatorCombo(this.m_Core);
        elevatorDownCombo = new UNIVERSAL_BringElevatorDownCombo(this.m_Core);
        stackUpCombo = new UNIVERSAL_AutoExitCombo(this.m_Core);
        capstoneOn = false;
    }

    @Override
    public void hardwareDestroy() {
        this.stoneOrientCombo = null;
        this.capstoneCombo = null;
        this.m_Core.saveAll();
    }

    @Override
    public void RunThisOpMode() {
        this.getRobotCore().readAll();
        this.getRobotCore().setDragServoToDrag(false);
        while(this.opModeIsActive()) {
            driveControl();
            foundationGraberControl();
            resetSlideControl();
            slideControl();
            grabberControl();
            grabberRotControl();
            intakeControl();
            capstoneControl();
            orientServoControl();
            ElevatorUpControl();
            ElevatorDownControl();
            StackUpControl();

            this.updateStatus();

            this.getRobotCore().updateTelemetry();
            telemetry.update();
        }
    }

    protected void driveControl(){
        boolean driveActivated = false;
        if(Math.abs(gamepad1.left_stick_x) >= LindelSettings.CONTROL_STICK_THRESHOLD || Math.abs(gamepad1.left_stick_y) >= LindelSettings.CONTROL_STICK_THRESHOLD || Math.abs(gamepad1.right_stick_x) >= LindelSettings.CONTROL_STICK_THRESHOLD){
            if(this.stackUpCombo.isBusy()){
                this.stackUpCombo.stopCombo();
            }
            driveActivated = true;
        }else{
            driveActivated = false;
        }

        if(driveActivated) {
            double xControl = -gamepad1.left_stick_y;
            double yControl = -gamepad1.left_stick_x;
            double rotControl = -gamepad1.right_stick_x;
            if (gamepad1.left_bumper) {
                xControl *= 0.25;
                yControl *= 0.25;
                rotControl *= 0.25;
            }
            driveTask.xSpeedNormalized = xControl * LindelSettings.CONTROL_CHASSIS_MAXSPEED_NORMALIZED * this.SpeedFactor;
            driveTask.ySpeedNormalized = yControl * LindelSettings.CONTROL_CHASSIS_MAXSPEED_NORMALIZED * this.SpeedFactor;
            driveTask.zRotSpeedNormalized = rotControl * LindelSettings.CONTROL_CHASSIS_MAXSPEED_NORMALIZED * this.SpeedFactor;
        }else{
            driveTask.xSpeedNormalized = 0;
            driveTask.ySpeedNormalized = 0;
            driveTask.zRotSpeedNormalized = 0;
        }
    }

    protected void foundationGraberControl(){
        if(gamepad1.right_bumper){
            this.getRobotCore().setDragServoToDrag(true);
        }else{
            this.m_Core.setDragServoToDrag(false);
        }
    }

    protected void slideControl(){
        if(Math.abs(gamepad2.left_stick_y) >= LindelSettings.CONTROL_STICK_THRESHOLD){
            if(this.stackUpCombo.isBusy()){
                this.stackUpCombo.stopCombo();
            }
            if(this.elevatorUpCombo.isBusy()){
                this.elevatorUpCombo.stopCombo();
            }
            if(this.elevatorDownCombo.isBusy()){
                this.elevatorDownCombo.stopCombo();
            }
            double targetY = -gamepad2.left_stick_y;
            RobotServoUsingMotorTask currentTask = this.m_Core.getLinearSlide().getCurrentTask();
            TargetPosTask currentSpecificTask = currentTask == null || !(currentTask instanceof TargetPosTask) ? null : (TargetPosTask) currentTask;
            double slideSpeed = Math.max(Math.abs(targetY), 0.5) * LindelSettings.CONTROL_SLIDE_MAXSPEED;
            if(targetY > 0){
                if(currentSpecificTask != null && currentSpecificTask.getTargetPos() == this.m_Core.getLinearSlide().getMaxPos()){
                    currentSpecificTask.setPower(slideSpeed);
                }else{
                    TargetPosTask newTask = new TargetPosTask(null,this.m_Core.getLinearSlide().getMaxPos(),slideSpeed);
                    this.m_Core.getLinearSlide().replaceTask(newTask);
                }
            }else{ //targetY < 0
                if(currentSpecificTask != null && currentSpecificTask.getTargetPos() == this.m_Core.getLinearSlide().getMinPos()){
                    currentSpecificTask.setPower(slideSpeed);
                }else{
                    TargetPosTask newTask = new TargetPosTask(null,this.m_Core.getLinearSlide().getMinPos(),slideSpeed);
                    this.m_Core.getLinearSlide().replaceTask(newTask);
                }
            }
        }else if(this.m_Core.getLinearSlide().isBusy()){
            if(!(this.stackUpCombo.isBusy() || this.elevatorUpCombo.isBusy() || this.elevatorDownCombo.isBusy())){
                this.m_Core.getLinearSlide().deleteAllTasks();
            }
        }
    }

    protected void grabberControl(){
        if (gamepad2.right_bumper) {
            if(this.stoneOrientCombo.isBusy()){
                this.stoneOrientCombo.stopCombo();
            }
            if(this.elevatorUpCombo.isBusy()){
                this.elevatorUpCombo.stopCombo();
            }
            if(this.stackUpCombo.isBusy()){
                this.stackUpCombo.stopCombo();
            }
            this.m_Core.setGrabberServoToGrab(false);
        } else {
            if(!(stoneOrientCombo.isBusy() || elevatorUpCombo.isBusy() || stackUpCombo.isBusy())){
                this.m_Core.setGrabberServoToGrab(true);
            }
        }
    }

    protected void grabberRotControl(){
        if(gamepad2.right_trigger >= LindelSettings.CONTROL_STICK_THRESHOLD || gamepad2.left_trigger >= LindelSettings.CONTROL_STICK_THRESHOLD) {
            if(elevatorUpCombo.isBusy()){
                elevatorUpCombo.stopCombo();
            }
            if(elevatorDownCombo.isBusy()){
                elevatorDownCombo.stopCombo();
            }
            if(stackUpCombo.isBusy()){
                stackUpCombo.stopCombo();
            }
            if (gamepad2.right_trigger >= LindelSettings.CONTROL_STICK_THRESHOLD) {
                if(!capstoneOn) {
                    this.m_Core.setGrabberRotServoToOutside(true, 1.0);
                }else{
                    this.m_Core.setGrabberRotServoToOutside(true, 0.5);
                }
            } else if (gamepad2.left_trigger >= LindelSettings.CONTROL_STICK_THRESHOLD) {
                this.m_Core.setGrabberRotServoToOutside(false, 1.0);
            }
        }
    }

    protected void intakeControl(){
        if(gamepad1.right_trigger >= LindelSettings.CONTROL_STICK_THRESHOLD){
            this.m_Core.setIntakeSystemStatus(LindelCore.IntakeSystemStatus.SUCK,gamepad1.right_trigger * LindelSettings.INTAKEMOTOR_SPEED);
            keepSucking = true;
        }else if(gamepad1.left_trigger >= LindelSettings.CONTROL_STICK_THRESHOLD){
            this.m_Core.setIntakeSystemStatus(LindelCore.IntakeSystemStatus.VOMIT,gamepad1.left_trigger * LindelSettings.INTAKEMOTOR_SPEED);
            keepSucking = false;
        }else{
            if(keepSucking){
                this.m_Core.setIntakeSystemStatus(LindelCore.IntakeSystemStatus.SUCK,LindelSettings.INTAKEMOTOR_SPEED);
            }else {
                this.m_Core.setIntakeSystemStatus(LindelCore.IntakeSystemStatus.STOP,0);
            }
        }
    }

    protected void capstoneControl(){
        if(gamepad2.x && (!capstoneCombo.isBusy())){
            this.capstoneCombo.startCombo();
            this.capstoneOn = true;
        }
    }

    protected void orientServoControl(){
        if(gamepad2.a && (!stoneOrientCombo.isBusy())){
            if(elevatorUpCombo.isBusy()){
                elevatorUpCombo.stopCombo();
            }
            if(stackUpCombo.isBusy()){
                stackUpCombo.stopCombo();
            }
            this.stoneOrientCombo.startCombo();
        }
    }

    protected void ElevatorUpControl(){
        if(gamepad2.dpad_up && (!elevatorUpCombo.isBusy())){
            if(elevatorDownCombo.isBusy()){
                elevatorDownCombo.stopCombo();
            }
            if(stackUpCombo.isBusy()){
                stackUpCombo.stopCombo();
            }
            if(stoneOrientCombo.isBusy()){
                stoneOrientCombo.stopCombo();
            }
            elevatorUpCombo.targetGrabberRotSpeed = this.capstoneOn ? 0.5 : 1.0;
            elevatorUpCombo.targetSlideSpeed = LindelSettings.CONTROL_SLIDE_MAXSPEED;
            double newStonePosition = lastStonePosition + LindelSettings.CONTROL_STONE_HEIGHT_SLIDE;
            if(lastStonePosition >= this.m_Core.getLinearSlide().getMinPos() && lastStonePosition <= this.m_Core.getLinearSlide().getMaxPos()) {
                newStonePosition = Range.clip(newStonePosition, this.m_Core.getLinearSlide().getMinPos(), this.m_Core.getLinearSlide().getMaxPos());
            }
            elevatorUpCombo.targetPosition = newStonePosition;
            elevatorUpCombo.startCombo();
        }
    }

    protected void ElevatorDownControl(){
        if(gamepad2.y && (!elevatorDownCombo.isBusy())){
            if(elevatorUpCombo.isBusy()){
                elevatorUpCombo.stopCombo();
            }
            if(stackUpCombo.isBusy()){
                stackUpCombo.stopCombo();
            }
            elevatorDownCombo.targetGrabberRotSpeed = 1.0;
            elevatorDownCombo.targetSlideSpeed = LindelSettings.CONTROL_SLIDE_MAXSPEED;
            lastStonePosition = this.m_Core.getLinearSlide().getCurrentPosition();
            elevatorDownCombo.startCombo();
        }
    }

    protected void StackUpControl(){
        if(gamepad2.dpad_down && (!stackUpCombo.isBusy())){
            if(elevatorDownCombo.isBusy()){
                elevatorDownCombo.stopCombo();
            }
            if(elevatorUpCombo.isBusy()){
                elevatorUpCombo.stopCombo();
            }
            if(stoneOrientCombo.isBusy()){
                stoneOrientCombo.stopCombo();
            }
            stackUpCombo.targetSlideSpeed = LindelSettings.CONTROL_SLIDE_MAXSPEED;
            stackUpCombo.targetChassisSpeed_Normalized = LindelSettings.CONTROL_COMBO_MAXIMUM_SPEED_NORMALIZED;
            stackUpCombo.targetChassisMaximumAccel_Normalized = LindelSettings.CONTROL_COMBO_MAXIMUM_ACCEL_NORMALIZED;
            stackUpCombo.startCombo();
        }
    }

    protected void resetSlideControl(){
        if(gamepad2.right_stick_button){
            if(this.stackUpCombo.isBusy()){
                this.stackUpCombo.stopCombo();
            }
            if(this.elevatorUpCombo.isBusy()){
                this.elevatorUpCombo.stopCombo();
            }
            if(this.elevatorDownCombo.isBusy()){
                this.elevatorDownCombo.stopCombo();
            }
            double currentPos = this.m_Core.getLinearSlide().getCurrentPosition();
            lastStonePosition -= currentPos;
            if(this.m_Core.getLinearSlide().isBusy()){
                this.m_Core.getLinearSlide().deleteAllTasks();
            }
            this.m_Core.getLinearSlide().adjustCurrentPosition(0);
        }
    }

    public void updateStatus(){
        this.capstoneCombo.updateStatus();
        this.stoneOrientCombo.updateStatus();
        this.elevatorUpCombo.updateStatus();
        this.elevatorDownCombo.updateStatus();
        this.stackUpCombo.updateStatus();

        this.m_Core.updateStatus();
    }


}
