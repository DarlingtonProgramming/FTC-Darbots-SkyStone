package org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
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
import org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.ElysiumAutoCore;
import org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.ElysiumCore;
import org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.Elysium_Settings.ElysiumAutonomousSettings;
import org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.Elysium_Settings.ElysiumSettings;
import org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.Soundboxes.ElysiumAutoSoundBox;
import org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.Subsystems.ElysiumAutoArm;
import org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.Subsystems.ElysiumStacker;

import java.util.List;

public abstract class ElysiumAutoBase extends DarbotsBasicOpMode<ElysiumAutoCore> {
    public static final RobotPoint2D placeStoneOnFoundationPosition_RED = new RobotPoint2D(
            SkyStoneCoordinates.FOUNDATION_RED.X,
            SkyStoneCoordinates.FOUNDATION_RED.Y - SkyStoneCoordinates.FOUNDATION_WIDTH / 2.0 - ElysiumSettings.PHYSICAL_CENTER_TO_RIGHT_SIGN
    );
    public static final RobotPoint2D placeStoneOnFoundationPosition_BLUE = new RobotPoint2D(
            SkyStoneCoordinates.FOUNDATION_BLUE.X,
            SkyStoneCoordinates.FOUNDATION_BLUE.Y + SkyStoneCoordinates.FOUNDATION_WIDTH / 2.0 + ElysiumSettings.PHYSICAL_CENTER_TO_LEFT_SIGN
    );
    public static final RobotPoint2D grabFoundationPosition_RED = new RobotPoint2D(
            SkyStoneCoordinates.FOUNDATION_RED.X,
            SkyStoneCoordinates.FOUNDATION_RED.Y - SkyStoneCoordinates.FOUNDATION_WIDTH / 2.0 - ElysiumSettings.PHYSICAL_CENTER_TO_BACK
    );
    public static final RobotPoint2D grabFoundationPosition_BLUE = new RobotPoint2D(
            SkyStoneCoordinates.FOUNDATION_BLUE.X,
            SkyStoneCoordinates.FOUNDATION_BLUE.Y + SkyStoneCoordinates.FOUNDATION_WIDTH / 2.0 + ElysiumSettings.PHYSICAL_CENTER_TO_BACK
    );
    public ElysiumAutoSoundBox autoSoundBox;
    @Override
    public void hardwareInitialize() {
        this.__hardwareInit();
        this.autoSoundBox = new ElysiumAutoSoundBox(this,this.getRobotCore());
        this.autoSoundBox.onInitialize();
    }

    public void grabLeftClaw(){
        this.getRobotCore().autoArmsSubSystem.leftArm.setGrabberServoState(ElysiumAutoArm.GrabberServoState.GRAB_STONE);
        autoSoundBox.onGrabbingStone();
        delay(ElysiumSettings.AUTONOMOUS_CLAW_WAIT_FOR_CLOSE_SEC);
        this.getRobotCore().autoArmsSubSystem.leftArm.setArmRotServoState(ElysiumAutoArm.ArmRotServoState.IN);
        delay(ElysiumSettings.AUTONOMOUS_CLAW_WAIT_FOR_OUT_IN_SEC);
    }

    public void grabRightClaw(){
        this.getRobotCore().autoArmsSubSystem.rightArm.setGrabberServoState(ElysiumAutoArm.GrabberServoState.GRAB_STONE);
        autoSoundBox.onGrabbingStone();
        delay(ElysiumSettings.AUTONOMOUS_CLAW_WAIT_FOR_CLOSE_SEC);
        this.getRobotCore().autoArmsSubSystem.rightArm.setArmRotServoState(ElysiumAutoArm.ArmRotServoState.IN);
        //delay(ElysiumSettings.AUTONOMOUS_CLAW_WAIT_FOR_OUT_IN_SEC);
    }
    public void setLeftClawToDropStone(){
        //this.getRobotCore().autoArmsSubSystem.leftArm.setArmRotServoState(ElysiumAutoArm.ArmRotServoState.OUT);
        this.getRobotCore().autoArmsSubSystem.leftArm.setGrabberServoState(ElysiumAutoArm.GrabberServoState.WIDE_OPEN);
        autoSoundBox.onReleasingStone();
    }

    public void setRightClawToDropStone(){
        //this.getRobotCore().autoArmsSubSystem.rightArm.setArmRotServoState(ElysiumAutoArm.ArmRotServoState.OUT);
        this.getRobotCore().autoArmsSubSystem.rightArm.setGrabberServoState(ElysiumAutoArm.GrabberServoState.WIDE_OPEN);
        autoSoundBox.onReleasingStone();
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
        this.autoSoundBox.onGrabbingFoundation();
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

    public static RobotPoint2D getLoadingZoneFurtherFromBridgePoint(AllianceType allianceType, ParkPosition parkPosition){
        RobotPoint2D closeToBridge = getLoadingZoneNextToBridgePoint(allianceType,parkPosition);
        closeToBridge.X -= SkyStoneCoordinates.NEUTRAL_BRIDGE_FLOOR_THINKNESS / 2.0;
        return closeToBridge;
    }

    public static RobotPoint2D getBuildingZoneFurtherFromBridgePoint(AllianceType allianceType, ParkPosition parkPosition){
        RobotPoint2D closeToBridge = getBuildingZoneNextToBridgePoint(allianceType,parkPosition);
        closeToBridge.X += SkyStoneCoordinates.NEUTRAL_BRIDGE_FLOOR_THINKNESS / 2.0;
        return closeToBridge;
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
    }
    public static Pose2d getRoadRunnerSplinePose(RobotPoint2D point){
        return XYPlaneCalculations.getPosefromDarbotsToRoadRunner(new RobotPose2D(point,0));
    }
    public static Vector2d getRoadRunnerPos(RobotPoint2D point){
        return XYPlaneCalculations.getPositionFromDarbotsToRoadRunner(point);
    }
}
