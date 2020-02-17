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
import org.darbots.darbotsftclib.libcore.purepursuit.followers.PurePursuitPathFollower;
import org.darbots.darbotsftclib.libcore.purepursuit.followers.PurePursuitWorldAxisFollower;
import org.darbots.darbotsftclib.libcore.purepursuit.waypoints.PurePursuitEndPoint;
import org.darbots.darbotsftclib.libcore.purepursuit.waypoints.PurePursuitHeadingInterpolationWayPoint;
import org.darbots.darbotsftclib.libcore.purepursuit.waypoints.PurePursuitWayPoint;
import org.darbots.darbotsftclib.libcore.runtime.GlobalUtil;
import org.darbots.darbotsftclib.libcore.runtime.MovementUtil;
import org.darbots.darbotsftclib.libcore.sensors.cameras.RobotOnPhoneCamera;
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
    public static final double DURATION_FOUNDATION = 0;
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
            pointAwayFromWall.Y += 20;
            //let's go away from the wall first.
            this.m_Core.chassis.followTrajectorySync(
                    this.m_Core.chassis.trajectoryBuilder()
                            .lineTo(XYPlaneCalculations.getPositionFromDarbotsToRoadRunner(pointAwayFromWall),new ConstantInterpolator(0))
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
        if(currentPosition.X >= 0){
            //gotta pass the bridge first
            this.setRightClawToRest();

            RobotPoint2D firstPoint = ElysiumAutoBase.getBuildingZoneFurtherFromBridgePoint(ALLIANCE_TYPE,PARK_POSITION);
            firstPoint.Y -= 10;
            RobotPoint2D secondPoint = ElysiumAutoBase.getLoadingZoneFurtherFromBridgePoint(ALLIANCE_TYPE,PARK_POSITION);
            secondPoint.Y -= 10;

            Trajectory trajectory = this.m_Core.chassis.trajectoryBuilder()
                    .splineTo(getRoadRunnerSplinePose(firstPoint),new ConstantInterpolator(Math.toRadians(-180)))
                    .lineTo(getRoadRunnerPos(secondPoint),new ConstantInterpolator(Math.toRadians(-180)))
                    .build();
            this.m_Core.chassis.followTrajectorySync(trajectory);
        }
        {
            //go to the stone.
            setRightClawToPrepareGrab();
            RobotPoint2D stonePosition = allStonePositions.get(stoneNumber - 1);
            Trajectory trajectory = this.m_Core.chassis.trajectoryBuilder()
                    .lineTo(getRoadRunnerPos(stonePosition),new ConstantInterpolator(Math.toRadians(-180)))
                    .build();
            this.m_Core.chassis.followTrajectorySync(trajectory);
        }
        {
            //grab stone
            this.grabRightClaw();
        }
        {
            //finished grabbing stone, now we should head to the foundation.
            ArrayList<PurePursuitWayPoint> wayPoints = new ArrayList<>();
            wayPoints.ensureCapacity(4);

            RobotPoint2D firstPoint = ElysiumAutoBase.getLoadingZoneFurtherFromBridgePoint(ALLIANCE_TYPE,PARK_POSITION);
            firstPoint.Y -= 5;

            RobotPoint2D secondPoint = ElysiumAutoBase.getLoadingZoneNextToBridgePoint(ALLIANCE_TYPE,PARK_POSITION);
            secondPoint.Y -= 10;

            RobotPoint2D thirdPoint = ElysiumAutoBase.getBuildingZoneFurtherFromBridgePoint(ALLIANCE_TYPE,PARK_POSITION);
            thirdPoint.Y -= 10;

            RobotPoint2D fourthPoint = ElysiumAutoBase.placeStoneOnFoundationPosition_RED;

            Trajectory trajectory = this.m_Core.chassis.trajectoryBuilder()
                    .splineTo(getRoadRunnerSplinePose(firstPoint),new ConstantInterpolator(Math.toRadians(-180)))
                    .lineTo(getRoadRunnerPos(secondPoint),new ConstantInterpolator(Math.toRadians(-180)))
                    .lineTo(getRoadRunnerPos(thirdPoint),new ConstantInterpolator(Math.toRadians(-180)))
                    .lineTo(getRoadRunnerPos(fourthPoint),new ConstantInterpolator(Math.toRadians(-180)))
                    .build();
            this.m_Core.chassis.followTrajectorySync(trajectory);
        }
        {
            //release stone from hand
            super.setRightClawToDropStone();
        }
        return true;
    }
    public boolean grabFoundationAndPark(){
        RobotPose2D currentPosition = this.getRobotCore().getCurrentPosition();
        double gotoFoundationPrepPositinoStartSpeed = 0.1;
        this.setRightClawToRest();
        if(currentPosition.X <= 0){
            //We are on the loading zone, so let's just go pass the bridge.
            ArrayList<PurePursuitWayPoint> wayPoints = new ArrayList<>();
            wayPoints.ensureCapacity(2);

            PurePursuitWayPoint firstPoint = new PurePursuitWayPoint(ElysiumAutoBase.getLoadingZoneFurtherFromBridgePoint(ALLIANCE_TYPE,PARK_POSITION));
            firstPoint.setEndFollowNormalizedSpeed(0.4);
            wayPoints.add(firstPoint);

            PurePursuitWayPoint secondPoint = new PurePursuitWayPoint(ElysiumAutoBase.getBuildingZoneFurtherFromBridgePoint(ALLIANCE_TYPE,PARK_POSITION));
            secondPoint.setEndFollowNormalizedSpeed(0.5);
            wayPoints.add(secondPoint);

            PurePursuitWorldAxisFollower goUnderBrdigeFollower = new PurePursuitWorldAxisFollower(wayPoints,0.1,0.5,ElysiumAutonomousSettings.PURE_PURSUIT_ANGLE_SPEED,-180);
            this.getRobotCore().getChassis().replaceTask(goUnderBrdigeFollower);
            if(!this.waitForDrive_WithTelemetry()){
                return false;
            }
            gotoFoundationPrepPositinoStartSpeed = 0.5;
        }
        {
            //raise up foundation grabber, go to foundation prep position, then go to the foundation
            super.prepareToGrabFoundation(ElysiumAutonomousSettings.STACKER_SLIDE_SPEED);
            ArrayList<PurePursuitWayPoint> wayPoints = new ArrayList<>();
            wayPoints.ensureCapacity(2);

            PurePursuitWayPoint firstPoint = new PurePursuitEndPoint(ElysiumAutoBase.grabFoundationPosition_RED,true,-180);
            firstPoint.setEndFollowNormalizedSpeed(0.4);
            firstPoint.Y -= 30;
            wayPoints.add(firstPoint);

            PurePursuitWayPoint secondPoint = new PurePursuitEndPoint(ElysiumAutoBase.grabFoundationPosition_RED,true,-90);
            secondPoint.Y += 5;
            secondPoint.setEndFollowNormalizedSpeed(0.05);
            wayPoints.add(secondPoint);

            PurePursuitWorldAxisFollower gotoFoundationFollower = new PurePursuitWorldAxisFollower(wayPoints,gotoFoundationPrepPositinoStartSpeed,0.4,ElysiumAutonomousSettings.PURE_PURSUIT_ANGLE_SPEED,-180);
            this.getRobotCore().getChassis().replaceTask(gotoFoundationFollower);
            if(!this.waitForDrive_WithTelemetry()){
                return false;
            }
        }
        {
            //let's grab the foundation and pull it out!
            super.grabFoundation(ElysiumAutonomousSettings.STACKER_SLIDE_SPEED);
            //foundation grabbing is a blocking function, so it the foundation is in our hands!
            //let's schedule a path for the robot to drag the foundation into place

            //first give the foundation a little bit of space to turn.
            this.getRobotCore().getChassis().replaceTask(MovementUtil.getGoToPointTask(
                    20,
                    -10,
                    0.1 * this.getRobotCore().getChassis().calculateMaxLinearSpeedInCMPerSec(),
                    0.5 * this.getRobotCore().getChassis().calculateMaxLinearSpeedInCMPerSec(),
                    0.1 * this.getRobotCore().getChassis().calculateMaxLinearSpeedInCMPerSec(),
                    0));
            if(!this.waitForDrive_WithTelemetry()){
                return false;
            }
            //let's turn and smash the foundation into the building site!
            currentPosition = this.getRobotCore().getCurrentPosition();
            this.getRobotCore().getChassis().replaceTask(MovementUtil.getTurnToWorldAngTask(
                    currentPosition,
                    -180,
                    0.1 * this.getRobotCore().getChassis().calculateMaxAngularSpeedInDegPerSec(),
                    0.5 * this.getRobotCore().getChassis().calculateMaxAngularSpeedInDegPerSec(),
                    0.1 * this.getRobotCore().getChassis().calculateMaxAngularSpeedInDegPerSec(),
                    true,
                    false
            ));
            this.getRobotCore().getChassis().addTask(MovementUtil.getGoToPointTask(
                    -20,
                    0,
                    0.1 * this.getRobotCore().getChassis().calculateMaxLinearSpeedInCMPerSec(),
                    0.5 * this.getRobotCore().getChassis().calculateMaxLinearSpeedInCMPerSec(),
                    0.1 * this.getRobotCore().getChassis().calculateMaxLinearSpeedInCMPerSec(),
                    0
            ));
            if(!this.waitForDrive_WithTelemetry()){
                return false;
            }

            //done with foundation, time to park!
            ArrayList<PurePursuitWayPoint> wayPoints = new ArrayList<>();
            wayPoints.ensureCapacity(2);

            PurePursuitWayPoint firstPoint = new PurePursuitWayPoint(ElysiumAutoBase.getBuildingZoneFurtherFromBridgePoint(ALLIANCE_TYPE,PARK_POSITION));
            firstPoint.setEndFollowNormalizedSpeed(0.5);
            wayPoints.add(firstPoint);

            PurePursuitWayPoint secondPoint = new PurePursuitEndPoint(SkyStoneCoordinates.getParkPosition(ALLIANCE_TYPE,PARK_POSITION),true,-180);
            secondPoint.setEndFollowNormalizedSpeed(0.1);
            wayPoints.add(secondPoint);

            PurePursuitWorldAxisFollower gotoParkFollower = new PurePursuitWorldAxisFollower(wayPoints,0.1,0.5,ElysiumAutonomousSettings.PURE_PURSUIT_ANGLE_SPEED,0);
            this.getRobotCore().getChassis().replaceTask(gotoParkFollower);
            if(!this.waitForDrive_WithTelemetry()){
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
