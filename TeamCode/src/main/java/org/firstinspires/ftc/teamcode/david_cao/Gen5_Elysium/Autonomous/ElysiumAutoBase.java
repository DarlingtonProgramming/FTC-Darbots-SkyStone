package org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.darbots.darbotsftclib.game_specific.AllianceType;
import org.darbots.darbotsftclib.libcore.OpModes.DarbotsBasicOpMode;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPoint2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPose2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.XYPlaneCalculations;
import org.darbots.darbotsftclib.libcore.odometry.DistanceSensorEnhancedOdometry;
import org.darbots.darbotsftclib.libcore.runtime.GlobalUtil;
import org.darbots.darbotsftclib.libcore.sensors.distance_sensors.DarbotsRevDistanceSensor;
import org.darbots.darbotsftclib.libcore.tasks.servo_tasks.motor_powered_servo_tasks.TargetPosTask;
import org.darbots.darbotsftclib.libcore.templates.sensors.DarbotsDistanceSensor;
import org.darbots.darbotsftclib.season_specific.skystone.ParkPosition;
import org.darbots.darbotsftclib.season_specific.skystone.SkyStoneCoordinates;
import org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.ElysiumAutoCore;
import org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.Elysium_Settings.ElysiumAutonomousSettings;
import org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.Elysium_Settings.ElysiumSettings;
import org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.Soundboxes.ElysiumAutoSoundBox;
import org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.Subsystems.ElysiumAutoArm;
import org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.Subsystems.ElysiumStacker;

import java.util.HashMap;
import java.util.Vector;

@Config
public abstract class ElysiumAutoBase extends DarbotsBasicOpMode<ElysiumAutoCore> {
    public static RobotPoint2D placeStoneOnFoundationPosition_RED = new RobotPoint2D(
            SkyStoneCoordinates.FOUNDATION_RED.X - 20 + (20 * 2),
            SkyStoneCoordinates.FOUNDATION_RED.Y - (SkyStoneCoordinates.FOUNDATION_WIDTH / 2.0 + ElysiumSettings.PHYSICAL_CENTER_TO_RIGHT_SIGN - 20)
    );
    public static RobotPoint2D placeStoneOnFoundationPosition_BLUE = new RobotPoint2D(
            SkyStoneCoordinates.FOUNDATION_BLUE.X - 20 + (20 * 2 - 5),
            SkyStoneCoordinates.FOUNDATION_BLUE.Y + (SkyStoneCoordinates.FOUNDATION_WIDTH / 2.0 + ElysiumSettings.PHYSICAL_CENTER_TO_LEFT_SIGN - 24)
    );
    public static RobotPoint2D grabFoundationPosition_RED = new RobotPoint2D(
            SkyStoneCoordinates.FOUNDATION_RED.X,
            SkyStoneCoordinates.FOUNDATION_RED.Y - (SkyStoneCoordinates.FOUNDATION_WIDTH / 2.0 + ElysiumSettings.PHYSICAL_CENTER_TO_BACK - 20)
    );
    public static RobotPoint2D grabFoundationPosition_BLUE = new RobotPoint2D(
            SkyStoneCoordinates.FOUNDATION_BLUE.X,
            SkyStoneCoordinates.FOUNDATION_BLUE.Y + (SkyStoneCoordinates.FOUNDATION_WIDTH / 2.0 + ElysiumSettings.PHYSICAL_CENTER_TO_BACK - 26)
    );
    public static double BridgeFurtherOffset = 0;

    public ElysiumAutoSoundBox autoSoundBox;
    @Override
    public void hardwareInitialize() {
        this.__hardwareInit();
        this.autoSoundBox = new ElysiumAutoSoundBox(this,this.getRobotCore());
        this.autoSoundBox.onInitialize();
    }
    public abstract DistanceSensorEnhancedOdometry.DistanceSensorOdometerSwitchType getDistanceSensorSwitchType(RobotPose2D currentPosition);

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
    public void setLeftClawToPrepareDrop(){
        this.getRobotCore().autoArmsSubSystem.leftArm.setArmRotServoState(ElysiumAutoArm.ArmRotServoState.PREPARE_DROP);
    }
    public void setRightClawToPrepareDrop(){
        this.getRobotCore().autoArmsSubSystem.rightArm.setArmRotServoState(ElysiumAutoArm.ArmRotServoState.PREPARE_DROP);
    }
    public void setLeftClawToDropStone(){
        //this.getRobotCore().autoArmsSubSystem.leftArm.setArmRotServoState(ElysiumAutoArm.ArmRotServoState.OUT);
        autoSoundBox.onReleasingStone();
        this.getRobotCore().autoArmsSubSystem.leftArm.setGrabberServoState(ElysiumAutoArm.GrabberServoState.WIDE_OPEN);
        delay(0.3);
    }

    public void setRightClawToDropStone(){
        //this.getRobotCore().autoArmsSubSystem.rightArm.setArmRotServoState(ElysiumAutoArm.ArmRotServoState.OUT);
        autoSoundBox.onReleasingStone();
        this.getRobotCore().autoArmsSubSystem.rightArm.setGrabberServoState(ElysiumAutoArm.GrabberServoState.WIDE_OPEN);
        delay(0.3);
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
        this.getRobotCore().stackerSubSystem.stackerSlide.replaceTask(new TargetPosTask(null, ElysiumSettings.STACKER_ABOVE_FOUNDATION_POS,power));
        while(this.getRobotCore().stackerSubSystem.stackerSlide.isBusy()){
            this.updateStatus();
        }
    }

    public void grabFoundation(double power){
        this.getRobotCore().chassis.followTrajectorySync(
                this.getRobotCore().chassis.trajectoryBuilder()
                .back(5)
                .build()
        );
        this.getRobotCore().stackerSubSystem.stackerSlide.replaceTask(new TargetPosTask(null,this.getRobotCore().stackerSubSystem.stackerSlide.getMinPos(),power));
        while(this.getRobotCore().stackerSubSystem.stackerSlide.isBusy() && this.opModeIsActive()){
            this.getRobotCore().stackerSubSystem.stackerSlide.updateStatus();
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
        closeToBridge.X -= SkyStoneCoordinates.NEUTRAL_BRIDGE_FLOOR_WIDTH / 2.0 + BridgeFurtherOffset;
        return closeToBridge;
    }

    public static RobotPoint2D getBuildingZoneFurtherFromBridgePoint(AllianceType allianceType, ParkPosition parkPosition){
        RobotPoint2D closeToBridge = getBuildingZoneNextToBridgePoint(allianceType,parkPosition);
        closeToBridge.X += SkyStoneCoordinates.NEUTRAL_BRIDGE_FLOOR_WIDTH / 2.0 + BridgeFurtherOffset;
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
        //time to close doors
        this.autoSoundBox.onStart();
        this.getRobotCore().stackerSubSystem.setDoorState(ElysiumStacker.Stacker_Door_State.CLOSED);
        __RunOpMode();
    }

    public abstract void __RunOpMode();

    public void updateStatus(){
        this.getRobotCore().updateStatus();
    }
    public static Pose2d getRoadRunnerSplinePose(RobotPoint2D point){
        return new Pose2d(point.X, point.Y,0);
    }

    public static Vector2d getRoadRunnerPos(RobotPoint2D point){
        return new Vector2d(point.X,point.Y);
    }

    public void calibratePositionUsingDistanceSensor(){

        if(!ElysiumAutonomousSettings.DISTANCE_SENSOR_POSITION_CALIBRATION){
            return;
        }
        RobotPose2D currentPose = this.getRobotCore().getCurrentPosition();
        DistanceSensorEnhancedOdometry.DistanceSensorOdometerSwitchType switchType = this.getDistanceSensorSwitchType(currentPose);
        if(switchType == DistanceSensorEnhancedOdometry.DistanceSensorOdometerSwitchType.NONE){
            return;
        }
        double squaredAngle = XYPlaneCalculations.roundDegToSquare(currentPose.getRotationZ());
        double angleError = squaredAngle - currentPose.getRotationZ();
        if(Math.abs(angleError) > 5){
            return;
        }
        if(switchType == DistanceSensorEnhancedOdometry.DistanceSensorOdometerSwitchType.BOTH_XY || switchType == DistanceSensorEnhancedOdometry.DistanceSensorOdometerSwitchType.ONLY_X) {
            if (currentPose.X >= 0) {
                DarbotsDistanceSensor sensor = null;
                double sensorPosition = 0;
                if (squaredAngle == 0) {
                    sensor = this.getRobotCore().FrontSensor.Sensor;
                    sensorPosition = this.getRobotCore().FrontSensor.OnRobotPosition.X;
                } else if (squaredAngle == 90) {
                    sensor = this.getRobotCore().RightSensor.Sensor;
                    sensorPosition = -this.getRobotCore().RightSensor.OnRobotPosition.Y;
                } else if (squaredAngle == -90) {
                    sensor = this.getRobotCore().LeftSensor.Sensor;
                    sensorPosition = this.getRobotCore().LeftSensor.OnRobotPosition.Y;
                } else { //squareAngle == -180
                    sensor = this.getRobotCore().BackSensor.Sensor;
                    sensorPosition = -this.getRobotCore().BackSensor.OnRobotPosition.X;
                }
                this.useSensorToCalibratePositiveXPosition(currentPose, sensorPosition, sensor, angleError);
            } else {
                DarbotsDistanceSensor sensor = null;
                double sensorPosition = 0;
                if (squaredAngle == 0) {
                    sensor = this.getRobotCore().BackSensor.Sensor;
                    sensorPosition = -this.getRobotCore().BackSensor.OnRobotPosition.X;
                } else if (squaredAngle == 90) {
                    sensor = this.getRobotCore().LeftSensor.Sensor;
                    sensorPosition = this.getRobotCore().LeftSensor.OnRobotPosition.Y;
                } else if (squaredAngle == -90) {
                    sensor = this.getRobotCore().RightSensor.Sensor;
                    sensorPosition = -this.getRobotCore().RightSensor.OnRobotPosition.Y;
                } else { //squareAngle == -180
                    sensor = this.getRobotCore().FrontSensor.Sensor;
                    sensorPosition = this.getRobotCore().FrontSensor.OnRobotPosition.X;
                }
                this.useSensorToCalibrateNegativeXPosition(currentPose, sensorPosition, sensor, angleError);
            }
        }
        if(switchType == DistanceSensorEnhancedOdometry.DistanceSensorOdometerSwitchType.BOTH_XY || switchType == DistanceSensorEnhancedOdometry.DistanceSensorOdometerSwitchType.ONLY_Y) {
            if (currentPose.Y >= 0) {
                DarbotsDistanceSensor sensor = null;
                double sensorPosition = 0;
                if (squaredAngle == 0) {
                    sensor = this.getRobotCore().LeftSensor.Sensor;
                    sensorPosition = this.getRobotCore().LeftSensor.OnRobotPosition.Y;
                } else if (squaredAngle == 90) {
                    sensor = this.getRobotCore().FrontSensor.Sensor;
                    sensorPosition = this.getRobotCore().FrontSensor.OnRobotPosition.X;
                } else if (squaredAngle == -90) {
                    sensor = this.getRobotCore().BackSensor.Sensor;
                    sensorPosition = -this.getRobotCore().BackSensor.OnRobotPosition.X;
                } else { //squareAngle == -180
                    sensor = this.getRobotCore().RightSensor.Sensor;
                    sensorPosition = -this.getRobotCore().RightSensor.OnRobotPosition.Y;
                }
                this.useSensorToCalibratePositiveYPosition(currentPose, sensorPosition, sensor, angleError);
            } else {
                DarbotsDistanceSensor sensor = null;
                double sensorPosition = 0;
                if (squaredAngle == 0) {
                    sensor = this.getRobotCore().RightSensor.Sensor;
                    sensorPosition = -this.getRobotCore().RightSensor.OnRobotPosition.Y;
                } else if (squaredAngle == 90) {
                    sensor = this.getRobotCore().BackSensor.Sensor;
                    sensorPosition = -this.getRobotCore().BackSensor.OnRobotPosition.X;
                } else if (squaredAngle == -90) {
                    sensor = this.getRobotCore().FrontSensor.Sensor;
                    sensorPosition = this.getRobotCore().FrontSensor.OnRobotPosition.X;
                } else { //squareAngle == -180
                    sensor = this.getRobotCore().LeftSensor.Sensor;
                    sensorPosition = this.getRobotCore().LeftSensor.OnRobotPosition.Y;
                }
                this.useSensorToCalibrateNegativeYPosition(currentPose, sensorPosition, sensor, angleError);
            }
        }
        this.getRobotCore().setCurrentPosition(currentPose);
    }

    @Override
    public void waitForStart(){
        while ((!opModeIsActive()) && (!isStopRequested())) {
            this.getRobotCore().updateStatus();
            this.calibratePositionUsingDistanceSensor();
            this.telemetry.addData("status","Initialized, waiting for start command...");
            TelemetryPacket packet = this.updateTelemetry();
            GlobalUtil.addTelmetryLine(null,packet,"status", "Initialized, waiting for start command...");
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
            telemetry.update();
        }
    }

    public void useSensorToCalibratePositiveXPosition(RobotPose2D positionReceiver, double sensorOnBotPosition, DarbotsDistanceSensor distanceSensor, double errorAngle){
        if(!ElysiumAutonomousSettings.DISTANCE_SENSOR_POSITION_CALIBRATION){
            return;
        }
        distanceSensor.updateStatus();
        double dist = distanceSensor.getDistanceInCM();
        if(dist == DarbotsDistanceSensor.DISTANCE_INVALID){
            return;
        }
        positionReceiver.X = SkyStoneCoordinates.FIELD_SIZE_X / 2.0 - Math.cos(Math.toRadians(errorAngle)) * (sensorOnBotPosition + dist);
        return;
    }

    public void useSensorToCalibrateNegativeXPosition(RobotPose2D positionReceiver, double sensorOnBotPosition, DarbotsDistanceSensor distanceSensor, double errorAngle){
        if(!ElysiumAutonomousSettings.DISTANCE_SENSOR_POSITION_CALIBRATION){
            return;
        }
        this.useSensorToCalibratePositiveXPosition(positionReceiver,sensorOnBotPosition,distanceSensor,errorAngle);
        positionReceiver.X = -positionReceiver.X;
    }

    public void useSensorToCalibratePositiveYPosition(RobotPose2D positionReceiver, double sensorOnBotPosition, DarbotsDistanceSensor distanceSensor, double errorAngle){
        if(!ElysiumAutonomousSettings.DISTANCE_SENSOR_POSITION_CALIBRATION){
            return;
        }
        distanceSensor.updateStatus();
        double dist = distanceSensor.getDistanceInCM();
        if(dist == DarbotsDistanceSensor.DISTANCE_INVALID){
            return;
        }
        positionReceiver.Y = SkyStoneCoordinates.FIELD_SIZE_Y / 2.0 - Math.cos(Math.toRadians(errorAngle)) * (sensorOnBotPosition + dist);
        return;
    }

    public void useSensorToCalibrateNegativeYPosition(RobotPose2D positionReceiver, double sensorOnBotPosition, DarbotsDistanceSensor distanceSensor, double errorAngle){
        if(!ElysiumAutonomousSettings.DISTANCE_SENSOR_POSITION_CALIBRATION){
            return;
        }
        this.useSensorToCalibratePositiveYPosition(positionReceiver,sensorOnBotPosition,distanceSensor,errorAngle);
        positionReceiver.Y = -positionReceiver.Y;
    }

    public boolean waitForDrive(){
        while(this.getRobotCore().chassis.isBusy() && this.opModeIsActive()){
            this.getRobotCore().chassis.update();
        }
        return this.opModeIsActive();
    }
}
