package org.firstinspires.ftc.teamcode.david_cao.Gen2_Lindel_Code;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.darbots.darbotsftclib.game_specific.AllianceType;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPoint2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPose2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPose3D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.XYPlaneCalculations;
import org.darbots.darbotsftclib.libcore.motion_planning.followers.TrajectoryFollower;
import org.darbots.darbotsftclib.libcore.purepursuit.followers.PurePursuitPathFollower;
import org.darbots.darbotsftclib.libcore.purepursuit.utils.PurePursuitWayPoint;
import org.darbots.darbotsftclib.libcore.runtime.MovementUtil;
import org.darbots.darbotsftclib.libcore.sensors.cameras.RobotOnPhoneCamera;
import org.darbots.darbotsftclib.libcore.tasks.chassis_tasks.FixedTurnAngleTask;
import org.darbots.darbotsftclib.libcore.tasks.chassis_tasks.FixedXDistanceTask;
import org.darbots.darbotsftclib.libcore.tasks.chassis_tasks.FixedYDistanceTask;
import org.darbots.darbotsftclib.season_specific.skystone.ParkPosition;
import org.darbots.darbotsftclib.season_specific.skystone.SkyStoneCoordinates;
import org.darbots.darbotsftclib.season_specific.skystone.SkyStonePosition;
import org.darbots.darbotsftclib.season_specific.skystone.darbots_pixel_skystone_detection.DarbotsPixelSkyStoneSampler;
import org.darbots.darbotsftclib.season_specific.skystone.darbots_vuforia_skystone_detection.DarbotsSkyStoneDifferentiation;
import org.darbots.darbotsftclib.season_specific.skystone.tfod_detection.SkyStoneStoneDifferentiation;
import org.firstinspires.ftc.teamcode.robot_common.Robot4100Common;

import java.util.ArrayList;

@Autonomous(group = "4100", name = "Lindel-Red-LoadingZone")
public class LindelRedLoadingZoneAuto extends LindelAutoBase {
    public static final AllianceType ALLIANCE = AllianceType.RED;
    public static final ParkPosition PARK_POSITION = ParkPosition.NEXT_TO_NEUTRAL_BRIDGE;
    public static final RobotPose2D FOUNDATION_POSE = new RobotPose2D(
            SkyStoneCoordinates.FOUNDATION_RED.X - LindelSettings.PHYSICAL_WIDTH / 2.0 + 20,
            SkyStoneCoordinates.FOUNDATION_RED.Y - SkyStoneCoordinates.FOUNDATION_WIDTH / 2.0 - LindelSettings.LENGTH_FROM_CENTER_TO_BACK,
            -90
    );
    public static final RobotPose2D FOUNDATION_FINISH_POSE = new RobotPose2D(
            FOUNDATION_POSE.X,
            -SkyStoneCoordinates.FIELD_SIZE_Y / 2.0 + LindelSettings.LENGTH_FROM_CENTER_TO_FRONT,
            -90
    );

    protected DarbotsPixelSkyStoneSampler m_Sampler;
    protected RobotOnPhoneCamera m_Camera;
    protected SkyStonePosition m_SampleResult;
    protected TrajectoryFollower[] m_EndingFollowers = new TrajectoryFollower[4];
    protected TrajectoryFollower[] m_GotoFoundationFirstTrajctoty = new TrajectoryFollower[2];
    protected TrajectoryFollower[] m_GotoFoundationTrajectory = new TrajectoryFollower[2];

    public ArrayList<RobotPoint2D> getStoneSamplePoints(int stoneNumber, double robotLength, double robotWidth){
        double halfRobotLength = robotLength / 2.0;
        double halfRobotWidth = robotWidth / 2.0;

        double deltaY = SkyStoneCoordinates.STONE_WIDTH / 2 + halfRobotLength;
        if(ALLIANCE == AllianceType.RED){
            deltaY = -deltaY;
        }
        double deltaX = SkyStoneCoordinates.STONE_LENGTH / 2 + halfRobotLength + 10;
        double lastDeltaX = -(halfRobotLength - SkyStoneCoordinates.STONE_LENGTH / 2);
        deltaX = -deltaX;
        lastDeltaX = -lastDeltaX;

        RobotPoint2D stonePos = SkyStoneCoordinates.getStonePosition(ALLIANCE,stoneNumber);

        RobotPoint2D firstPoint = new RobotPoint2D(stonePos.X + deltaX,stonePos.Y + deltaY);
        RobotPoint2D secondPoint = new RobotPoint2D(stonePos.X + deltaX,stonePos.Y);
        RobotPoint2D thirdPoint = new RobotPoint2D(stonePos.X + lastDeltaX, stonePos.Y);
        ArrayList<RobotPoint2D> returnList = new ArrayList<>();
        returnList.add(firstPoint);
        returnList.add(secondPoint);
        returnList.add(thirdPoint);
        return returnList;
    }

    @Override
    public void RunThisOpMode() {
        //Set Torch Mode
        RobotOnPhoneCamera.setFlashlightEnabled(LindelSettings.AUTO_SAMPLE_LIGHT);
        //Try to Sample
        this.m_SampleResult = m_Sampler.sample(
                LindelSampleInfos.SAMPLE_PICTURE_SIZE_X,
                LindelSampleInfos.SAMPLE_PICTURE_SIZE_Y,
                LindelSampleInfos.SAMPLE_SCALED_SIZE_X,
                LindelSampleInfos.SAMPLE_SCALED_SIZE_Y,
                LindelSampleInfos.RED_WALL_STONE_START_X,
                LindelSampleInfos.RED_WALL_STONE_START_Y,
                LindelSampleInfos.RED_WALL_STONE_END_X,
                LindelSampleInfos.RED_WALL_STONE_END_Y,
                LindelSampleInfos.RED_CENTER_STONE_START_X,
                LindelSampleInfos.RED_CENTER_STONE_START_Y,
                LindelSampleInfos.RED_CENTER_STONE_END_X,
                LindelSampleInfos.RED_CENTER_STONE_END_Y,
                LindelSampleInfos.RED_BRIDGE_STONE_START_X,
                LindelSampleInfos.RED_BRIDGE_STONE_START_Y,
                LindelSampleInfos.RED_BRIDGE_STONE_END_X,
                LindelSampleInfos.RED_BRIDGE_STONE_END_Y
        );
        //Grab First Stone Based On Sample Result
        int firstStoneNumber = 0;
        if(m_SampleResult == SkyStonePosition.NEXT_TO_BRIDGE){
            firstStoneNumber = 1;
        }else if(m_SampleResult == SkyStonePosition.MIDDLE){
            firstStoneNumber = 2;
        }else{
            firstStoneNumber = 3;
        }

        this.updateTelemetry();
        telemetry.update();

        //Retrieve First Stone WayPoints
        {
            ArrayList<RobotPoint2D> firstStoneXYPosition = this.getStoneSamplePoints(firstStoneNumber, LindelSettings.PHYSICAL_LENGTH, LindelSettings.PHYSICAL_WIDTH);
            //start Driving and take in stones
            /* for(int i = 0; i<firstStoneXYPosition.size() - 1; i++){
                RobotPoint2D currentPoint = firstStoneXYPosition.get(i);
                if(firstStoneNumber==2){
                    currentPoint.X += 20;
                    currentPoint.Y += 10;
                }
                RobotPose2D currentRobotAxisPoint = this.getRobotPose(new RobotPose2D(currentPoint,0));
                double preferredAngle = this.getRobotCore().getChassis().getPreferredAngle(0);
                this.getRobotCore().getChassis().replaceTask(MovementUtil.getGoToPointTask(currentRobotAxisPoint.X,currentRobotAxisPoint.Y,0,0.5 * this.getRobotCore().getChassis().calculateMaxLinearSpeedInCMPerSec(),0,preferredAngle));
                if (!this.waitForDrive_WithTelemetry()) {
                    return;
                }
            }
            this.startSuckStones();
            RobotPoint2D currentPoint = firstStoneXYPosition.get(firstStoneXYPosition.size() - 1);
            if(firstStoneNumber==2){
                currentPoint.X += 20;
                currentPoint.Y += 10;
            }
            RobotPose2D currentRobotAxisPoint = this.getRobotPose(new RobotPose2D(currentPoint,0));
            double preferredAngle = this.getRobotCore().getChassis().getPreferredAngle(0);
            this.getRobotCore().getChassis().replaceTask(MovementUtil.getGoToPointTask(currentRobotAxisPoint.X,currentRobotAxisPoint.Y,0,0.5 * this.getRobotCore().getChassis().calculateMaxLinearSpeedInCMPerSec(),0,preferredAngle));
            if (!this.waitForDrive_WithTelemetry()) {
                return;
            }
             */
            if(firstStoneNumber == 1){
                firstStoneXYPosition.get(0).X += 5;
                firstStoneXYPosition.get(1).X += 5;
            }else if(firstStoneNumber == 2){
                firstStoneXYPosition.get(0).X += 2;
                firstStoneXYPosition.get(0).Y += 3;
                firstStoneXYPosition.get(1).X += 2;
                firstStoneXYPosition.get(1).Y += 3;
            }else{
                this.getRobotCore().getChassis().replaceTask(MovementUtil.getGoToPointTask(10,35,0.3 * this.getRobotCore().getChassis().calculateMaxLinearSpeedInCMPerSec(),0.3 * this.getRobotCore().getChassis().calculateMaxLinearSpeedInCMPerSec(),0.2 * this.getRobotCore().getChassis().calculateMaxLinearSpeedInCMPerSec(),0));
                if(!waitForDrive()){
                    return;
                }
                firstStoneXYPosition.get(0).X -= 10;
                firstStoneXYPosition.get(1).X -= 10;
                firstStoneXYPosition.get(1).Y += 5;
                firstStoneXYPosition.get(2).Y += 5;
            }

            RobotPose2D currentPosition = this.getRobotCore().getChassis().getCurrentPosition();
            this.startSuckStones();
            this.getRobotCore().getChassis().replaceTask(this.getFollower(this.transferWorldPointsToPurePursuitPoints(firstStoneXYPosition,currentPosition),LindelSettings.AUTO_PURE_PURSUIT_RADIUS,0.55));
            if(!this.waitForDrive_WithTelemetry()){
                return;
            }
        }

        //stop taking in stones and start driving under bridge
        {
            RobotPose2D currentRobotPosition;
            this.stopSuckStones();
            ArrayList<RobotPoint2D> firstStoneExitWayPointsRaw = SkyStoneCoordinates.getPurePursuitWayPointsExitWorldAxis(ALLIANCE, firstStoneNumber, LindelSettings.PHYSICAL_LENGTH, LindelSettings.PHYSICAL_WIDTH, PARK_POSITION);
            if(firstStoneNumber == 2 || firstStoneNumber == 3) {
                firstStoneExitWayPointsRaw.get(0).Y -= 30;
                firstStoneExitWayPointsRaw.get(1).Y -= 30;
            }else{
                firstStoneExitWayPointsRaw.get(0).Y -= 20;
                firstStoneExitWayPointsRaw.get(1).Y -= 20;
            }
            for(RobotPoint2D i : firstStoneExitWayPointsRaw){
                currentRobotPosition = this.getRobotCore().getChassis().getCurrentPosition();
                double preferredAngle = this.getRobotCore().getChassis().getPreferredAngle(0);
                RobotPose2D target = XYPlaneCalculations.getRelativePosition(currentRobotPosition,new RobotPose2D(i,preferredAngle));
                this.getRobotCore().getChassis().replaceTask(MovementUtil.getGoToPointTask(target.X,target.Y,0.3 * this.getRobotCore().getChassis().calculateMaxLinearSpeedInCMPerSec(),0.3 * this.getRobotCore().getChassis().calculateMaxLinearSpeedInCMPerSec(),0.3 * this.getRobotCore().getChassis().calculateMaxLinearSpeedInCMPerSec(),preferredAngle));
                if(!this.waitForDrive_WithTelemetry()){
                    return;
                }
            }
            if(firstStoneNumber != 1){
                RobotPose2D currentPosition = this.getRobotCore().PosTracker.getCurrentPosition();
                currentPosition.Y += 10;
                this.getRobotCore().PosTracker.setCurrentPosition(currentPosition);
            }
        }

        //go to foundation
        {
            TrajectoryFollower[] followArr = null;
            if(firstStoneNumber == 1){
                followArr = this.m_GotoFoundationFirstTrajctoty;
            }else{
                followArr = this.m_GotoFoundationTrajectory;
            }
            for(int i=0;i<followArr.length;i++){
                this.getRobotCore().getChassis().replaceTask(followArr[i]);
                if(!waitForDrive_WithTelemetry()){
                    return;
                }
            }
        }

        //grab foundation, put stone down, pull foundation
        {
            RobotPose2D currentRobotPosition = this.getRobotCore().getChassis().getCurrentPosition();
            double preferredAngle = this.getRobotCore().getChassis().getPreferredAngle(FOUNDATION_FINISH_POSE.getRotationZ());
            RobotPoint2D foundationFinishRobotAxis = XYPlaneCalculations.getRelativePosition(currentRobotPosition,FOUNDATION_FINISH_POSE.toPoint2D());
            TrajectoryFollower follower = MovementUtil.getGoToPointTask(foundationFinishRobotAxis.X + 70,foundationFinishRobotAxis.Y,0,0.3 * this.getRobotCore().getChassis().calculateMaxLinearSpeedInCMPerSec(),0.05 * this.getRobotCore().getChassis().calculateMaxLinearSpeedInCMPerSec(),preferredAngle);
            this.getRobotCore().setDragServoToDrag(true);
            this.depositStoneToFoundation();
            delay(0.3);
            this.getRobotCore().getChassis().replaceTask(follower);
            if(!this.waitForDrive_WithTelemetry()){
                return;
            }
            this.getRobotCore().setDragServoToDrag(false);
            currentRobotPosition = this.getRobotCore().getChassis().getCurrentPosition();
            currentRobotPosition.Y = FOUNDATION_FINISH_POSE.Y;
            this.getRobotCore().PosTracker.setCurrentPosition(currentRobotPosition);
            while(this.dropStoneToFoundationCombo.isBusy() && this.opModeIsActive()){
                this.updateStatus();
            }
        }

        //exit Foundation
        {
            for(int i=0;i<this.m_EndingFollowers.length;i++){
                this.getRobotCore().getChassis().replaceTask(this.m_EndingFollowers[i]);
                if(!this.waitForDrive_WithTelemetry()){
                    return;
                }
            }
        }
    }

    @Override
    public void __init() {
        m_SampleResult = SkyStonePosition.UNKNOWN;
        m_Camera = new RobotOnPhoneCamera(this,true, RobotOnPhoneCamera.PhoneCameraDirection.Back, Robot4100Common.VUFORIA_LICENSE);
        m_Sampler = new DarbotsPixelSkyStoneSampler(m_Camera);
        //Initial Position - Right side of the robot lined up with the end of 2nd Tile from the audience wall
        RobotPose2D initPose = new RobotPose2D(SkyStoneCoordinates.RED_LOADING_ZONE_FIELD_EXTREME_POINT,90);
        initPose.X += 119.38 - LindelSettings.PHYSICAL_WIDTH / 2;
        initPose.Y += LindelSettings.LENGTH_FROM_CENTER_TO_BACK;
        this.getRobotCore().PosTracker.setCurrentPosition(initPose);
        this.m_EndingFollowers[0] = MovementUtil.getGoToPointTask(0,-80,0.2 * this.getRobotCore().getChassis().calculateMaxLinearXSpeedInCMPerSec(),0.5 * this.getRobotCore().getChassis().calculateMaxLinearXSpeedInCMPerSec(),0.5 * this.getRobotCore().getChassis().calculateMaxLinearXSpeedInCMPerSec(),0);
        this.m_EndingFollowers[1] = MovementUtil.getGoToPointTask(-50,0,0.5 * this.getRobotCore().getChassis().calculateMaxLinearXSpeedInCMPerSec(),0.5 * this.getRobotCore().getChassis().calculateMaxLinearXSpeedInCMPerSec(),0.5 * this.getRobotCore().getChassis().calculateMaxLinearXSpeedInCMPerSec(),0);
        this.m_EndingFollowers[2] = MovementUtil.getGoToPointTask(0,40,0.5 * this.getRobotCore().getChassis().calculateMaxLinearXSpeedInCMPerSec(),0.5 * this.getRobotCore().getChassis().calculateMaxLinearXSpeedInCMPerSec(),0.3 * this.getRobotCore().getChassis().calculateMaxLinearXSpeedInCMPerSec(),0);
        this.m_EndingFollowers[3] = MovementUtil.getGoToPointTask(-5,-60,0.3 * this.getRobotCore().getChassis().calculateMaxLinearXSpeedInCMPerSec(),0.5 * this.getRobotCore().getChassis().calculateMaxLinearXSpeedInCMPerSec(),0,0);


        ArrayList<RobotPoint2D> exitPoint = SkyStoneCoordinates.getPurePursuitWayPointsExitWorldAxis(ALLIANCE,1,LindelSettings.PHYSICAL_LENGTH,LindelSettings.PHYSICAL_WIDTH,ParkPosition.NEXT_TO_NEUTRAL_BRIDGE);
        RobotPoint2D lastExitPoint = exitPoint.get(exitPoint.size() - 1);
        RobotPose2D expectedRobotPosition = new RobotPose2D(lastExitPoint,0);

        RobotPose2D currentRobotPosition = new RobotPose2D(expectedRobotPosition);
        currentRobotPosition.Y -= 20;

        RobotPoint2D foundationPosition1RobotAxis = XYPlaneCalculations.getRelativePosition(currentRobotPosition,new RobotPoint2D(FOUNDATION_POSE.X,currentRobotPosition.Y));

        TrajectoryFollower stone1Pos1Follower = MovementUtil.getGoToPointTask(foundationPosition1RobotAxis.X,foundationPosition1RobotAxis.Y,0.2 * this.getRobotCore().getChassis().calculateMaxLinearSpeedInCMPerSec(),0.3 * this.getRobotCore().getChassis().calculateMaxLinearSpeedInCMPerSec(),0.3 * this.getRobotCore().getChassis().calculateMaxLinearSpeedInCMPerSec(),-90);

        currentRobotPosition = XYPlaneCalculations.getAbsolutePosition(currentRobotPosition,new RobotPose2D(foundationPosition1RobotAxis,-90));
        RobotPoint2D foundationPosition2RobotAxis = XYPlaneCalculations.getRelativePosition(currentRobotPosition,FOUNDATION_POSE.toPoint2D());
        TrajectoryFollower stone1Pos2Follower = MovementUtil.getGoToPointTask(foundationPosition2RobotAxis.X,foundationPosition2RobotAxis.Y,0.3 * this.getRobotCore().getChassis().calculateMaxLinearSpeedInCMPerSec(),0.5 * this.getRobotCore().getChassis().calculateMaxLinearSpeedInCMPerSec(),0.05 * this.getRobotCore().getChassis().calculateMaxLinearSpeedInCMPerSec(),0);
        this.m_GotoFoundationFirstTrajctoty[0] = stone1Pos1Follower;
        this.m_GotoFoundationFirstTrajctoty[1] = stone1Pos2Follower;

        currentRobotPosition.setValues(expectedRobotPosition);
        currentRobotPosition.Y -= 30;

        RobotPoint2D foundationPosition23_1RobotAxis = XYPlaneCalculations.getRelativePosition(currentRobotPosition,new RobotPoint2D(FOUNDATION_POSE.X,currentRobotPosition.Y));

        TrajectoryFollower stone23Pos1Follower = MovementUtil.getGoToPointTask(foundationPosition23_1RobotAxis.X,foundationPosition23_1RobotAxis.Y,0.2 * this.getRobotCore().getChassis().calculateMaxLinearSpeedInCMPerSec(),0.3 * this.getRobotCore().getChassis().calculateMaxLinearSpeedInCMPerSec(),0.3 * this.getRobotCore().getChassis().calculateMaxLinearSpeedInCMPerSec(),-90);

        currentRobotPosition = XYPlaneCalculations.getAbsolutePosition(currentRobotPosition,new RobotPose2D(foundationPosition23_1RobotAxis,-90));
        RobotPoint2D foundationPosition23_2RobotAxis = XYPlaneCalculations.getRelativePosition(currentRobotPosition,FOUNDATION_POSE.toPoint2D());
        TrajectoryFollower stone23Pos2Follower = MovementUtil.getGoToPointTask(foundationPosition23_2RobotAxis.X,foundationPosition23_2RobotAxis.Y,0.3 * this.getRobotCore().getChassis().calculateMaxLinearSpeedInCMPerSec(),0.5 * this.getRobotCore().getChassis().calculateMaxLinearSpeedInCMPerSec(),0.05 * this.getRobotCore().getChassis().calculateMaxLinearSpeedInCMPerSec(),0);
        this.m_GotoFoundationTrajectory[0] = stone23Pos1Follower;
        this.m_GotoFoundationTrajectory[1] = stone23Pos2Follower;
    }

    @Override
    public void __destroy() {
        RobotOnPhoneCamera.setFlashlightEnabled(false);
    }

    @Override
    public void __updateTelemetry() {
        telemetry.addData("Sample Result",m_SampleResult.name());
    }
}
