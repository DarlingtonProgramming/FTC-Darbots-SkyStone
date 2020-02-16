package org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.Autonomous;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.darbots.darbotsftclib.game_specific.AllianceType;
import org.darbots.darbotsftclib.libcore.OpModes.DarbotsBasicOpMode;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPoint2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPose2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.XYPlaneCalculations;
import org.darbots.darbotsftclib.libcore.tasks.servo_tasks.motor_powered_servo_tasks.TargetPosTask;
import org.darbots.darbotsftclib.libcore.templates.DarbotsAction;
import org.darbots.darbotsftclib.season_specific.skystone.ParkPosition;
import org.darbots.darbotsftclib.season_specific.skystone.SkyStoneCoordinates;
import org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.ElysiumCore;
import org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.Elysium_Settings.ElysiumSettings;
import org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.Soundboxes.ElysiumAutoSoundBox;
import org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.Subsystems.ElysiumAutoArm;
import org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.Subsystems.ElysiumStacker;

public abstract class ElysiumAutoBase extends DarbotsBasicOpMode<ElysiumCore> {
    public static class ElysiumAutoClawCloseAction extends DarbotsAction{
        public ElysiumAutoArm m_Arm = null;
        private ElapsedTime m_Time = null;

        public ElysiumAutoClawCloseAction(ElysiumAutoArm arm){
            this.m_Arm = arm;
        }

        @Override
        protected void __startAction() {
            if(this.m_Time == null) {
                this.m_Time = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
            }{
                this.m_Time.reset();
            }
            this.m_Arm.setGrabberServoState(ElysiumAutoArm.GrabberServoState.GRAB_STONE);
        }

        @Override
        protected void __stopAction() {

        }

        public boolean rotServoReady(){
            if((!this.isBusy()) && this.m_Time.seconds() >= ElysiumSettings.AUTONOMOUS_CLAW_WAIT_FOR_OUT_IN_SEC){
                return true;
            }else{
                return false;
            }
        }

        @Override
        public void updateStatus() {
            if(this.isBusy()) {
                if (this.m_Time.seconds() >= ElysiumSettings.AUTONOMOUS_CLAW_WAIT_FOR_CLOSE_SEC){
                    this.m_Arm.setArmRotServoState(ElysiumAutoArm.ArmRotServoState.IN);
                    this.m_Time.reset();
                    this.stopAction();
                }
            }
        }
    }
    public static class ElysiumWaitForArmRotToComeInAction extends DarbotsAction{
        public ElysiumAutoClawCloseAction clawCloseAction;

        public ElysiumWaitForArmRotToComeInAction(ElysiumAutoClawCloseAction autoClawCloseAction){
            this.clawCloseAction = autoClawCloseAction;
        }

        @Override
        protected void __startAction() {

        }

        @Override
        protected void __stopAction() {

        }

        @Override
        public void updateStatus() {
            if(this.isBusy()) {
                if (this.clawCloseAction.rotServoReady()) {
                    this.stopAction();
                }
            }
        }
    }
    public ElysiumAutoSoundBox autoSoundBox;
    protected ElysiumAutoClawCloseAction m_LeftClawClose, m_RightClawClose;

    @Override
    public void hardwareInitialize() {
        this.__hardwareInit();
        this.autoSoundBox = new ElysiumAutoSoundBox(this);
        this.autoSoundBox.onInitialize();
        this.m_LeftClawClose = new ElysiumAutoClawCloseAction(this.getRobotCore().autoArmsSubSystem.leftArm);
        this.m_RightClawClose = new ElysiumAutoClawCloseAction(this.getRobotCore().autoArmsSubSystem.rightArm);
    }

    public void closeLeftClaw(){
        if(!this.m_LeftClawClose.isBusy()) {
            this.m_LeftClawClose.startAction();
            autoSoundBox.onGrabbingStone();
        }
    }

    public void closeRightClaw(){
        if(!this.m_RightClawClose.isBusy()) {
            this.m_RightClawClose.startAction();
            autoSoundBox.onGrabbingStone();
        }
    }

    public boolean isLeftClawClosed(){
        return (!this.m_LeftClawClose.isBusy());
    }

    public boolean isRightClawClosed(){
        return (!this.m_RightClawClose.isBusy());
    }

    public void setLeftClawToDropStone(){
        //this.getRobotCore().autoArmsSubSystem.leftArm.setArmRotServoState(ElysiumAutoArm.ArmRotServoState.OUT);
        this.getRobotCore().autoArmsSubSystem.leftArm.setGrabberServoState(ElysiumAutoArm.GrabberServoState.WIDE_OPEN);
    }

    public void setRightClawToDropStone(){
        //this.getRobotCore().autoArmsSubSystem.rightArm.setArmRotServoState(ElysiumAutoArm.ArmRotServoState.OUT);
        this.getRobotCore().autoArmsSubSystem.rightArm.setGrabberServoState(ElysiumAutoArm.GrabberServoState.WIDE_OPEN);
    }

    public void setLeftClawToPrepareGrab(){
        this.getRobotCore().autoArmsSubSystem.leftArm.setGrabberServoState(ElysiumAutoArm.GrabberServoState.WIDE_OPEN);
        this.getRobotCore().autoArmsSubSystem.leftArm.setArmRotServoState(ElysiumAutoArm.ArmRotServoState.OUT);
    }

    public void setRightClawToPrepareGrab(){
        this.getRobotCore().autoArmsSubSystem.rightArm.setGrabberServoState(ElysiumAutoArm.GrabberServoState.WIDE_OPEN);
        this.getRobotCore().autoArmsSubSystem.rightArm.setArmRotServoState(ElysiumAutoArm.ArmRotServoState.OUT);
    }

    public void setLeftClawToRest(){
        this.getRobotCore().autoArmsSubSystem.leftArm.setGrabberServoState(ElysiumAutoArm.GrabberServoState.CLOSED);
        this.getRobotCore().autoArmsSubSystem.leftArm.setArmRotServoState(ElysiumAutoArm.ArmRotServoState.REST);
    }

    public void setRightClawToRest(){
        this.getRobotCore().autoArmsSubSystem.rightArm.setGrabberServoState(ElysiumAutoArm.GrabberServoState.CLOSED);
        this.getRobotCore().autoArmsSubSystem.rightArm.setArmRotServoState(ElysiumAutoArm.ArmRotServoState.REST);
    }

    public void prepareToGrabFoundation(double power){
        this.getRobotCore().stackerSubSystem.stackerSlide.replaceTask(new TargetPosTask(null, ElysiumStacker.STACKER_SLIDE_ABOVE_FOUNDATION_POS,power));
    }

    public void grabFoundation(double power){
        this.getRobotCore().stackerSubSystem.stackerSlide.replaceTask(new TargetPosTask(null,ElysiumStacker.STACKER_SLIDE_MIN_POS,power));
        while(this.getRobotCore().stackerSubSystem.stackerSlide.isBusy()){
            this.updateStatus();
        }
    }

    public ElysiumWaitForArmRotToComeInAction getWaitForLeftClawInAction(){
        return new ElysiumWaitForArmRotToComeInAction(this.m_LeftClawClose);
    }

    public ElysiumWaitForArmRotToComeInAction getWaitForRightClawInAction(){
        return new ElysiumWaitForArmRotToComeInAction(this.m_RightClawClose);
    }

    public static RobotPose2D getGrabStonePose(AllianceType allianceType, int stoneNumber){
        double rotation = -180;
        RobotPoint2D stonePosition = SkyStoneCoordinates.getStonePosition(allianceType,stoneNumber);
        RobotPoint2D clawPosition = allianceType == AllianceType.BLUE ? ElysiumSettings.AUTONOMOUS_CLAW_LEFT_POSITION_WHEN_DOWN : ElysiumSettings.AUTONOMOUS_CLAW_RIGHT_POSITION_WHEN_DOWN;
        RobotPose2D supposedWorldRobotPose = XYPlaneCalculations.getAbsolutePosition(rotation,clawPosition,stonePosition);
        if(stoneNumber == 6){
            supposedWorldRobotPose.X = - (SkyStoneCoordinates.FIELD_SIZE_X / 2.0 - ElysiumSettings.PHYSICAL_CENTER_TO_FRONT);
        }
        return supposedWorldRobotPose;
    }

    public static RobotPoint2D getLoadingZoneNextToBridgePoint(AllianceType allianceType, ParkPosition parkPosition){
        RobotPoint2D parkPoint = SkyStoneCoordinates.getParkPosition(allianceType,parkPosition);
        RobotPoint2D LoadingZonePoint = new RobotPoint2D(parkPoint);
        LoadingZonePoint.X -= ElysiumSettings.PHYSICAL_CENTER_TO_BACK_DOOR_CLOSED;
        return LoadingZonePoint;
    }

    public static RobotPoint2D getBuildingZoneNextToBridgePoint(AllianceType allianceType, ParkPosition parkPosition){
        RobotPoint2D parkPoint = SkyStoneCoordinates.getParkPosition(allianceType,parkPosition);
        RobotPoint2D LoadingZonePoint = new RobotPoint2D(parkPoint);
        LoadingZonePoint.X += ElysiumSettings.PHYSICAL_CENTER_TO_FRONT;
        return LoadingZonePoint;
    }

    public abstract void __hardwareInit();

    @Override
    public void hardwareDestroy() {
        this.autoSoundBox.terminate();
        this.__hardwareDestroy();
    }

    public abstract void __hardwareDestroy();

    @Override
    public void RunThisOpMode() {
        this.autoSoundBox.onStart();
        __RunOpMode();
    }

    public abstract void __RunOpMode();

    public void updateStatus(){
        this.getRobotCore().updateStatus();
        this.m_LeftClawClose.updateStatus();
        this.m_RightClawClose.updateStatus();
    }
}
