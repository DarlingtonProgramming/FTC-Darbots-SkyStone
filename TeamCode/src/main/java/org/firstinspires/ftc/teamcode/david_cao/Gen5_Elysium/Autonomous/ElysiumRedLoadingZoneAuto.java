package org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.darbots.darbotsftclib.game_specific.AllianceType;
import org.darbots.darbotsftclib.libcore.OpModes.DarbotsBasicOpMode;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPoint2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPose2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.XYPlaneCalculations;
import org.darbots.darbotsftclib.libcore.purepursuit.waypoints.PurePursuitWayPoint;
import org.darbots.darbotsftclib.libcore.runtime.GlobalUtil;
import org.darbots.darbotsftclib.libcore.sensors.cameras.RobotOnPhoneCamera;
import org.darbots.darbotsftclib.libcore.tasks.servo_tasks.motor_powered_servo_tasks.TargetPosTask;
import org.darbots.darbotsftclib.libcore.templates.DarbotsAction;
import org.darbots.darbotsftclib.season_specific.skystone.ParkPosition;
import org.darbots.darbotsftclib.season_specific.skystone.SkyStoneCoordinates;
import org.darbots.darbotsftclib.season_specific.skystone.SkyStonePosition;
import org.darbots.darbotsftclib.season_specific.skystone.darbots_pixel_skystone_detection.DarbotsPixelSkyStoneSampler;
import org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.ElysiumAutoCore;
import org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.ElysiumCore;
import org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.Elysium_Settings.ElysiumAutonomousSettings;
import org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.Elysium_Settings.ElysiumSettings;
import org.firstinspires.ftc.teamcode.robot_common.Robot4100Common;

import java.lang.reflect.Array;
import java.util.ArrayList;

@Autonomous(group = "4100", name = "Elysium-Auto-Red-LoadingZone")
public class ElysiumRedLoadingZoneAuto extends ElysiumAutoBase {
    public static final AllianceType ALLIANCE_TYPE = AllianceType.RED;
    public static final ParkPosition PARK_POSITION = ParkPosition.NEXT_TO_NEUTRAL_BRIDGE;
    public static final double DURATION_EACH_STONE = 10.0;
    public static final double DURATION_FOUNDATION = 10.0;
    public static final double DURATION_PARK = 4.0;

    private ElysiumAutoCore m_Core;
    private int telemetry_I;
    private RobotOnPhoneCamera m_Camera;
    private DarbotsPixelSkyStoneSampler m_Sampler;
    private ArrayList<RobotPose2D> allStonePositions;
    private SkyStonePosition sampledPosition;

    @Override
    public ElysiumAutoCore getRobotCore() {
        return this.m_Core;
    }

    @Override
    public void __hardwareInit() {
        this.m_Core = new ElysiumAutoCore("ElysiumRedLoadingZoneAuto.log",this.hardwareMap,false, ElysiumAutonomousSettings.RED_AUTO_START_POSE,false);
        this.m_Camera = new RobotOnPhoneCamera(this,ElysiumAutonomousSettings.SAMPLE_PREVIEW, RobotOnPhoneCamera.PhoneCameraDirection.Back, Robot4100Common.VUFORIA_LICENSE);
        this.m_Sampler = new DarbotsPixelSkyStoneSampler(this.m_Camera);
        this.telemetry_I = 0;
        this.allStonePositions = new ArrayList<>();
        this.allStonePositions.ensureCapacity(6);
        for(int i = 1; i <= 6; i++){
            this.allStonePositions.add(ElysiumAutoBase.getGrabStonePose(ALLIANCE_TYPE,i));
        }
    }

    @Override
    public void __hardwareDestroy() {

    }

    @Override
    public void __RunOpMode() {
        //Sample First
        this.sampledPosition = m_Sampler.sample(
                ElysiumAutonomousSettings.SAMPLE_PICTURE_SIZE_X,
                ElysiumAutonomousSettings.SAMPLE_PICTURE_SIZE_Y,
                ElysiumAutonomousSettings.SAMPLE_SHRINKED_SIZE_X,
                ElysiumAutonomousSettings.SAMPLE_SHRINKED_SIZE_Y,
                ElysiumAutonomousSettings.SAMPLE_RED_STONE_NEXT_TO_WALL_START_X,
                ElysiumAutonomousSettings.SAMPLE_RED_STONE_NEXT_TO_WALL_START_Y,
                ElysiumAutonomousSettings.SAMPLE_RED_STONE_NEXT_TO_WALL_END_X,
                ElysiumAutonomousSettings.SAMPLE_RED_STONE_NEXT_TO_WALL_END_Y,
                ElysiumAutonomousSettings.SAMPLE_RED_STONE_MIDDLE_START_X,
                ElysiumAutonomousSettings.SAMPLE_RED_STONE_MIDDLE_START_Y,
                ElysiumAutonomousSettings.SAMPLE_RED_STONE_MIDDLE_END_X,
                ElysiumAutonomousSettings.SAMPLE_RED_STONE_MIDDLE_END_Y,
                ElysiumAutonomousSettings.SAMPLE_RED_STONE_NEXT_TO_BRIDGE_START_X,
                ElysiumAutonomousSettings.SAMPLE_RED_STONE_NEXT_TO_BRIDGE_START_Y,
                ElysiumAutonomousSettings.SAMPLE_RED_STONE_NEXT_TO_BRIDGE_END_X,
                ElysiumAutonomousSettings.SAMPLE_RED_STONE_NEXT_TO_BRIDGE_END_Y
        );
        //calculate which stones to grab first
        int[] skystoneNumbers = {sampledPosition.value(), sampledPosition.value() + 3};
        int[] remainingStoneNumbers = new int[4];
        int[] getStoneOrder = new int[6];
        {
            getStoneOrder[0] = skystoneNumbers[0];
            getStoneOrder[1] = skystoneNumbers[1];
            int remainingNumberCounter = 0;
            for (int i = 1; i <= 6; i++) {
                if (i != skystoneNumbers[0] && i != skystoneNumbers[1]) {
                    remainingStoneNumbers[remainingNumberCounter] = i;
                    getStoneOrder[remainingNumberCounter + 2] = i;
                    remainingNumberCounter++;
                }
            }
        }
        {
            RobotPoint2D pointAwayFromWall = ElysiumAutonomousSettings.RED_AUTO_START_POSE;
            pointAwayFromWall.Y += 30;
            //let's go away from the wall first.
            this.m_Core.chassis.followTrajectorySync(
                    this.m_Core.chassis.trajectoryBuilder()
                            .lineTo(getRoadRunnerPos(pointAwayFromWall),new ConstantInterpolator(0))
                    .build()
            );
            this.m_Core.chassis.turnSync(
                    Math.toRadians(180)
            );
        }

        {
            //do as many stones as possible
            for(int i=0; i<6; i++) {
                if ((30.0 - this.getSecondsSinceOpModeStarted() - DURATION_FOUNDATION) < DURATION_EACH_STONE) {
                    break;
                }
                this.runToStoneAndPlaceOnFoundation(getStoneOrder[i]);
            }
        }
        {
            //not much time left, left's grab the foundation and park
            if(!this.grabFoundationAndPark()){
                return;
            }
        }
    }

    public boolean runToStoneAndPlaceOnFoundation(int stoneNumber){
        RobotPose2D currentPosition = this.getRobotCore().getCurrentPosition();
        double grabStoneStartSpeed = 0.1;
        RobotPoint2D stonePosition = allStonePositions.get(stoneNumber - 1);
        if(currentPosition.X >= 0){
            //gotta pass the bridge first
            this.setRightClawToRest();

            RobotPoint2D firstPoint = ElysiumAutoBase.getBuildingZoneFurtherFromBridgePoint(ALLIANCE_TYPE,PARK_POSITION);
            firstPoint.Y -= 10;
            RobotPoint2D secondPoint = ElysiumAutoBase.getLoadingZoneFurtherFromBridgePoint(ALLIANCE_TYPE,PARK_POSITION);
            secondPoint.Y -= 10;

            RobotPoint2D thirdPoint = new RobotPoint2D(stonePosition);
            thirdPoint.Y -= 15;

            Trajectory trajectory = this.m_Core.chassis.trajectoryBuilder()
                    .lineTo(getRoadRunnerPos(firstPoint),new ConstantInterpolator(Math.toRadians(-180)))
                    .lineTo(getRoadRunnerPos(secondPoint),new ConstantInterpolator(Math.toRadians(-180)))
                    .lineTo(getRoadRunnerPos(thirdPoint),new ConstantInterpolator(Math.toRadians(-180)))
                    .build();
            this.m_Core.chassis.followTrajectorySync(trajectory);
            if(!this.opModeIsActive()){
                return false;
            }
            this.calibratePositionUsingDistanceSensor();
        } else{
            //go to the stone.
            setRightClawToPrepareGrab();
            Trajectory trajectory = this.m_Core.chassis.trajectoryBuilder()
                    .lineTo(getRoadRunnerPos(stonePosition),new ConstantInterpolator(Math.toRadians(-180)))
                    .build();
            this.m_Core.chassis.followTrajectorySync(trajectory);
            if(!this.opModeIsActive()){
                return false;
            }
            this.calibratePositionUsingDistanceSensor();
        }
        {
            //grab stone
            this.grabRightClaw();
            if(!this.opModeIsActive()){
                return false;
            }
        }
        {
            //finished grabbing stone, now we should head to the foundation.
            RobotPoint2D firstPoint = ElysiumAutoBase.getLoadingZoneFurtherFromBridgePoint(ALLIANCE_TYPE,PARK_POSITION);
            firstPoint.Y -= 10;

            RobotPoint2D secondPoint = ElysiumAutoBase.getLoadingZoneNextToBridgePoint(ALLIANCE_TYPE,PARK_POSITION);
            secondPoint.Y -= 15;

            RobotPoint2D thirdPoint = ElysiumAutoBase.getBuildingZoneFurtherFromBridgePoint(ALLIANCE_TYPE,PARK_POSITION);
            thirdPoint.Y -= 15;

            RobotPoint2D fourthPoint = ElysiumAutoBase.placeStoneOnFoundationPosition_RED;

            Trajectory trajectory = this.m_Core.chassis.trajectoryBuilder()
                    .lineTo(getRoadRunnerPos(firstPoint),new ConstantInterpolator(Math.toRadians(-180)))
                    .lineTo(getRoadRunnerPos(secondPoint),new ConstantInterpolator(Math.toRadians(-180)))
                    .lineTo(getRoadRunnerPos(thirdPoint),new ConstantInterpolator(Math.toRadians(-180)))
                    .lineTo(getRoadRunnerPos(fourthPoint),new ConstantInterpolator(Math.toRadians(-180)))
                    .build();
            this.m_Core.chassis.followTrajectorySync(trajectory);
            if(!this.opModeIsActive()){
                return false;
            }
            this.calibratePositionUsingDistanceSensor();
        }
        {
            //release stone from hand
            super.setRightClawToDropStone();
        }
        return true;
    }
    public boolean grabFoundationAndPark(){
        RobotPose2D currentPosition = this.getRobotCore().getCurrentPosition();
        if(currentPosition.X <= 0){
            //we are not in the building zone, let's go to the building zone.
            this.setRightClawToRest();
            RobotPoint2D firstPoint = ElysiumAutoBase.getLoadingZoneFurtherFromBridgePoint(ALLIANCE_TYPE,PARK_POSITION);
            firstPoint.Y -= 5;

            RobotPoint2D secondPoint = ElysiumAutoBase.getLoadingZoneNextToBridgePoint(ALLIANCE_TYPE,PARK_POSITION);
            secondPoint.Y -= 10;

            RobotPoint2D thirdPoint = ElysiumAutoBase.getBuildingZoneFurtherFromBridgePoint(ALLIANCE_TYPE,PARK_POSITION);
            thirdPoint.Y -= 10;

            Trajectory trajectory = this.m_Core.chassis.trajectoryBuilder()
                    .lineTo(getRoadRunnerPos(firstPoint),new ConstantInterpolator(Math.toRadians(-180)))
                    .lineTo(getRoadRunnerPos(secondPoint),new ConstantInterpolator(Math.toRadians(-180)))
                    .lineTo(getRoadRunnerPos(thirdPoint),new ConstantInterpolator(Math.toRadians(-180)))
                    .build();
            this.m_Core.chassis.followTrajectorySync(trajectory);

            if(!this.opModeIsActive()){
                return false;
            }
        }
        {
            //We are absolutely in the building zone now, go to position to prepare to grab foundation.
            //we can rise the foundation puller first
            this.getRobotCore().stackerSubSystem.stackerSlide.replaceTask(new TargetPosTask(null,this.getRobotCore().stackerSubSystem.STACKER_SLIDE_ABOVE_FOUNDATION_POS,ElysiumAutonomousSettings.STACKER_SLIDE_SPEED));
            while(this.getRobotCore().stackerSubSystem.stackerSlide.isBusy() && this.opModeIsActive()){
                this.getRobotCore().updateStatus();
                this.lazyUpdateTelemetry();
            }

            RobotPoint2D grabFoundationPos = ElysiumAutoBase.grabFoundationPosition_RED;
            RobotPoint2D grabFoundationPrepPos = new RobotPoint2D(grabFoundationPos);
            grabFoundationPos.Y -= 40;
            Trajectory trajectory =
                    this.getRobotCore().chassis.trajectoryBuilder()
                    .lineTo(getRoadRunnerPos(grabFoundationPrepPos),new ConstantInterpolator(Math.toRadians(-180)))
                    .lineTo(getRoadRunnerPos(grabFoundationPos),new ConstantInterpolator(Math.toRadians(-90)))
                    .build();
            this.m_Core.chassis.followTrajectorySync(trajectory);
            if(!this.opModeIsActive()){
                return false;
            }

            //set puller to grab foundation
            this.getRobotCore().stackerSubSystem.stackerSlide.replaceTask(new TargetPosTask(null,this.getRobotCore().stackerSubSystem.STACKER_SLIDE_MIN_POS,ElysiumAutonomousSettings.STACKER_SLIDE_SPEED));
            while(this.getRobotCore().stackerSubSystem.stackerSlide.isBusy() && this.opModeIsActive()){
                this.getRobotCore().updateStatus();
                this.lazyUpdateTelemetry();
            }
            this.calibratePositionUsingDistanceSensor();
        }
        {
            //Now the foundation is in our hands, let's pull it.
            RobotPoint2D firstPoint = new RobotPoint2D(ElysiumAutoBase.grabFoundationPosition_RED);
            firstPoint.X -= 15;
            firstPoint.Y = SkyStoneCoordinates.RED_BUILDING_ZONE_FIELD_EXTREME_POINT.Y + (SkyStoneCoordinates.TILE_FLOOR_CONNECTION_SIDE_WIDTH + ElysiumSettings.FOUNDATION_HOOK_DISTANCE_FROM_CENTER);
            RobotPoint2D secondPoint = new RobotPoint2D(firstPoint);
            secondPoint.X = SkyStoneCoordinates.RED_BUILDING_ZONE_FIELD_EXTREME_POINT.X - (SkyStoneCoordinates.FOUNDATION_WIDTH + ElysiumSettings.PHYSICAL_CENTER_TO_BACK);

            Trajectory trajectory =
                    this.getRobotCore().chassis.trajectoryBuilder()
                            .lineTo(getRoadRunnerPos(firstPoint),new ConstantInterpolator(Math.toRadians(-90)))
                            .lineTo(getRoadRunnerPos(firstPoint),new ConstantInterpolator(Math.toRadians(-180)))
                            .build();
            this.m_Core.chassis.followTrajectorySync(trajectory);
            if(!this.opModeIsActive()){
                return false;
            }
        }
        {
            //foundation pulled, let's get our hands out of the foundation
            this.getRobotCore().stackerSubSystem.stackerSlide.replaceTask(new TargetPosTask(null,this.getRobotCore().stackerSubSystem.STACKER_SLIDE_ABOVE_FOUNDATION_POS,ElysiumAutonomousSettings.STACKER_SLIDE_SPEED));
            while(this.getRobotCore().stackerSubSystem.stackerSlide.isBusy() && this.opModeIsActive()){
                this.getRobotCore().updateStatus();
                this.lazyUpdateTelemetry();
            }

            RobotPoint2D leaveFoundationPoint = new RobotPoint2D(this.getRobotCore().getCurrentPosition());
            leaveFoundationPoint.X -= 10;
            this.m_Core.chassis.followTrajectorySync(
                    this.m_Core.chassis.trajectoryBuilder()
                    .lineTo(getRoadRunnerPos(leaveFoundationPoint),new ConstantInterpolator(-180))
                    .build()
            );
            if(!this.opModeIsActive()){
                return false;
            }

            this.getRobotCore().stackerSubSystem.stackerSlide.replaceTask(new TargetPosTask(null,this.getRobotCore().stackerSubSystem.STACKER_SLIDE_MIN_POS,ElysiumAutonomousSettings.STACKER_SLIDE_SPEED));
            while(this.getRobotCore().stackerSubSystem.stackerSlide.isBusy() && this.opModeIsActive()){
                this.getRobotCore().updateStatus();
                this.lazyUpdateTelemetry();
            }
            this.calibratePositionUsingDistanceSensor();
        }
        {
            //finish foundation pulling, go to park
            RobotPoint2D firstPoint = ElysiumAutoBase.getBuildingZoneFurtherFromBridgePoint(ALLIANCE_TYPE,PARK_POSITION);
            firstPoint.Y -= 10;

            RobotPoint2D secondPoint = SkyStoneCoordinates.getParkPosition(ALLIANCE_TYPE,PARK_POSITION);
            secondPoint.Y -= 5;

            Trajectory trajectory = this.getRobotCore().chassis.trajectoryBuilder()
                    .lineTo(getRoadRunnerPos(firstPoint),new ConstantInterpolator(-180))
                    .lineTo(getRoadRunnerPos(secondPoint), new ConstantInterpolator(-180))
                    .build();
            this.m_Core.chassis.followTrajectorySync(trajectory);
            if(!this.opModeIsActive()){
                return false;
            }
        }
        return true;
    }


    public TelemetryPacket updateTelemetry(){
        TelemetryPacket packet = super.updateTelemetry();
        if(this.sampledPosition != null) {
            GlobalUtil.addTelmetryLine(this.telemetry, packet, "RecognitionResult", this.sampledPosition.name());
        }
        return packet;
    }

    public void lazyUpdateTelemetry(){
        if(telemetry_I >= DarbotsBasicOpMode.CONST_TELMETRY_PACKET_CYCLE_TIME){
            telemetry_I = 0;
            TelemetryPacket packet = this.updateTelemetry();
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
            this.telemetry.update();
        }else{
            telemetry_I++;
        }
    }
}
