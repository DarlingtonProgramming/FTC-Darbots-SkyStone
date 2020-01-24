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
            SkyStoneCoordinates.FOUNDATION_RED.X - SkyStoneCoordinates.FOUNDATION_LENGTH / 2.0 + LindelSettings.PHYSICAL_WIDTH / 2.0,
            SkyStoneCoordinates.FOUNDATION_RED.Y - SkyStoneCoordinates.FOUNDATION_WIDTH / 2.0 - LindelSettings.LENGTH_FROM_CENTER_TO_BACK,
            -90
    );
    public static final RobotPose2D FOUNDATION_FINISH_POSE = new RobotPose2D(
            FOUNDATION_POSE.X,
            -SkyStoneCoordinates.FIELD_SIZE_Y / 2.0 + LindelSettings.LENGTH_FROM_CENTER_TO_FRONT,
            -90
    );
    public static final RobotPose2D FOUNDATION_EXIT_POSE = new RobotPose2D(
            SkyStoneCoordinates.FOUNDATION_RED.X - SkyStoneCoordinates.FOUNDATION_LENGTH / 2.0 - (LindelSettings.PHYSICAL_WIDTH / 2.0 - 10),
            FOUNDATION_FINISH_POSE.Y,
            -90
    );

    protected DarbotsPixelSkyStoneSampler m_Sampler;
    protected RobotOnPhoneCamera m_Camera;
    protected SkyStonePosition m_SampleResult;

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

        //Retrieve First Stone WayPoints
        {
            RobotPose2D currentRobotPosition = this.getRobotCore().PosTracker.getCurrentPosition();
            ArrayList<RobotPoint2D> firstStoneWayPointsRaw = SkyStoneCoordinates.getPurePursuitWayPointsWorldAxis(ALLIANCE, firstStoneNumber, LindelSettings.PHYSICAL_LENGTH, LindelSettings.PHYSICAL_WIDTH, currentRobotPosition.toPoint2D());
            PurePursuitPathFollower firstStoneFollower = this.getFollower(this.transferWorldPointsToPurePursuitPoints(firstStoneWayPointsRaw, currentRobotPosition), LindelSettings.AUTO_PURE_PURSUIT_RADIUS);

            //start Driving and take in stones
            this.startSuckStones();
            this.getRobotCore().getChassis().replaceTask(firstStoneFollower);
            if (!this.waitForDrive()) {
                return;
            }
        }

        //stop taking in stones and start driving under bridge
        {
            RobotPose2D currentRobotPosition = this.getRobotCore().PosTracker.getCurrentPosition();
            this.stopSuckStones();
            ArrayList<RobotPoint2D> firstStoneExitWayPointsRaw = SkyStoneCoordinates.getPurePursuitWayPointsExitWorldAxis(ALLIANCE, firstStoneNumber, LindelSettings.PHYSICAL_LENGTH, LindelSettings.PHYSICAL_WIDTH, PARK_POSITION);
            PurePursuitPathFollower firstStoneExitFollower = this.getFollower(this.transferWorldPointsToPurePursuitPoints(firstStoneExitWayPointsRaw,currentRobotPosition),LindelSettings.AUTO_PURE_PURSUIT_RADIUS);
            this.getRobotCore().getChassis().replaceTask(firstStoneExitFollower);
            if(!this.waitForDrive()){
                return;
            }
        }

        //go to foundation
        {
            RobotPose2D currentRobotPosition = this.getRobotCore().PosTracker.getCurrentPosition();
            double preferredAngle = this.getRobotCore().getChassis().getPreferredAngle(FOUNDATION_POSE.getRotationZ());
            RobotPoint2D foundationPositionRobotAxis = XYPlaneCalculations.getRelativePosition(currentRobotPosition,FOUNDATION_POSE.toPoint2D());
            TrajectoryFollower follower = MovementUtil.getGoToPointTask(foundationPositionRobotAxis.X,foundationPositionRobotAxis.Y,0,this.maxAutoSpeed,0,preferredAngle);
            this.getRobotCore().getChassis().replaceTask(follower);
            if(!this.waitForDrive()){
                return;
            }
            //fix angle of robot after the previous job.
            this.getRobotCore().getChassis().replaceTask(MovementUtil.getTurnToWorldAngTask(FOUNDATION_POSE.getRotationZ(),0,this.maxAutoAngularSpeed,0,false,false));
            if(!this.waitForDrive()){
                return;
            }
        }

        //grab foundation, put stone down, pull foundation
        {
            RobotPose2D currentRobotPosition = this.getRobotCore().PosTracker.getCurrentPosition();
            double preferredAngle = this.getRobotCore().getChassis().getPreferredAngle(FOUNDATION_FINISH_POSE.getRotationZ());
            RobotPoint2D foundationFinishRobotAxis = XYPlaneCalculations.getRelativePosition(currentRobotPosition,FOUNDATION_FINISH_POSE.toPoint2D());
            TrajectoryFollower follower = MovementUtil.getGoToPointTask(foundationFinishRobotAxis.X,foundationFinishRobotAxis.Y,0,this.maxAutoSpeed,0,preferredAngle);
            this.getRobotCore().setDragServoToDrag(true);
            this.depositStoneToFoundation();
            this.getRobotCore().getChassis().replaceTask(follower);
            if(!this.waitForDrive()){
                return;
            }
            while(this.dropStoneToFoundationCombo.isBusy() && this.opModeIsActive()){
                this.updateStatus();
            }
        }

        //exit Foundation
        {
            RobotPose2D currentRobotPosition = this.getRobotCore().PosTracker.getCurrentPosition();
            double preferredAngle = this.getRobotCore().getChassis().getPreferredAngle(FOUNDATION_EXIT_POSE.getRotationZ());
            RobotPoint2D foundationExitRobotAxis = XYPlaneCalculations.getRelativePosition(currentRobotPosition,FOUNDATION_EXIT_POSE.toPoint2D());
            TrajectoryFollower follower = MovementUtil.getGoToPointTask(foundationExitRobotAxis.X,foundationExitRobotAxis.Y,0,this.maxAutoSpeed,0,preferredAngle);
            this.getRobotCore().setDragServoToDrag(false);
            this.getRobotCore().getChassis().replaceTask(follower);
            if(!this.waitForDrive()){
                return;
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
    }

    @Override
    public void __destroy() {
        RobotOnPhoneCamera.setFlashlightEnabled(false);
    }
}
