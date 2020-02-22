package org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.darbots.darbotsftclib.game_specific.AllianceType;
import org.darbots.darbotsftclib.libcore.OpModes.DarbotsBasicOpMode;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPoint2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPose2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotVector2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.XYPlaneCalculations;
import org.darbots.darbotsftclib.libcore.odometry.DistanceSensorEnhancedOdometry;
import org.darbots.darbotsftclib.libcore.runtime.GlobalUtil;
import org.darbots.darbotsftclib.libcore.sensors.cameras.RobotOnPhoneCamera;
import org.darbots.darbotsftclib.libcore.tasks.servo_tasks.motor_powered_servo_tasks.TargetPosTask;
import org.darbots.darbotsftclib.libcore.templates.odometry.RobotAsyncPositionTracker;
import org.darbots.darbotsftclib.season_specific.skystone.ParkPosition;
import org.darbots.darbotsftclib.season_specific.skystone.SkyStoneCoordinates;
import org.darbots.darbotsftclib.season_specific.skystone.SkyStonePosition;
import org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.ElysiumAutoCore;
import org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.Elysium_Settings.ElysiumAutonomousSettings;
import org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.Elysium_Settings.ElysiumSettings;
import org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.RoadRunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.RoadRunner.drive.localizer.RoadRunnerLocalizer;
import org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.Subsystems.ElysiumStacker;
import org.firstinspires.ftc.teamcode.robot_common.Robot4100Common;

import java.util.ArrayList;

import kotlin.Unit;

@Autonomous(group = "4100", name = "Elysium-Auto-Blue-LoadingZone-Secure")
public class ElysiumBlueLoadingZoneSecureAuto extends ElysiumAutoBase {
    public static final AllianceType ALLIANCE_TYPE = AllianceType.BLUE;
    public static final ParkPosition PARK_POSITION = ParkPosition.NEXT_TO_NEUTRAL_BRIDGE;
    public static final double DURATION_EACH_STONE = 10.0;
    public static final double DURATION_FOUNDATION = 10.0;
    public static final double DURATION_PARK = 4.0;

    private ElysiumAutoCore m_Core;
    private int telemetry_I;
    private RobotOnPhoneCamera m_Camera;
    private ElysiumAutoSampler m_Sampler;
    private ArrayList<RobotPose2D> allStonePositions;
    private SkyStonePosition sampledPosition;
    private RobotPoint2D lastDropStonePos;

    public static DriveConstraints secureConstraints = new DriveConstraints(
            50 / XYPlaneCalculations.INCH_PER_CM, 30 / XYPlaneCalculations.INCH_PER_CM,0,
            Math.toRadians(180),Math.toRadians(135),0
    );

    @Override
    public ElysiumAutoCore getRobotCore() {
        return this.m_Core;
    }

    @Override
    public DistanceSensorEnhancedOdometry.DistanceSensorOdometerSwitchType getDistanceSensorSwitchType(RobotPose2D currentPosition) {
        if(currentPosition.X >= ElysiumAutonomousSettings.DISTANCE_SENSOR_CALIBRATION_STARTX && currentPosition.X <= ElysiumAutonomousSettings.DISTANCE_SENSOR_CALIBRATION_ENDX){
            return DistanceSensorEnhancedOdometry.DistanceSensorOdometerSwitchType.BOTH_XY;
        }else{
            return DistanceSensorEnhancedOdometry.DistanceSensorOdometerSwitchType.ONLY_X;
        }
    }

    @Override
    public void __hardwareInit() {
        this.m_Core = new ElysiumAutoCore("ElysiumRedLoadingZoneAuto.log",this.hardwareMap,false, ElysiumAutonomousSettings.BLUE_AUTO_START_POSE,false);
        this.m_Core.chassis.constraints = new MecanumConstraints(secureConstraints, DriveConstants.TRACK_WIDTH);
        this.m_Camera = new RobotOnPhoneCamera(this,ElysiumAutonomousSettings.SAMPLE_PREVIEW, RobotOnPhoneCamera.PhoneCameraDirection.Back, Robot4100Common.VUFORIA_LICENSE);
        this.m_Sampler = new ElysiumAutoSampler(this.m_Camera);
        this.telemetry_I = 0;
        this.allStonePositions = new ArrayList<>();
        this.allStonePositions.ensureCapacity(6);
        for(int i = 1; i <= 6; i++){
            this.allStonePositions.add(ElysiumAutoBase.getGrabStonePose(ALLIANCE_TYPE,i));
        }
        this.lastDropStonePos = ElysiumAutoBase.placeStoneOnFoundationPosition_BLUE;
    }

    @Override
    public void __hardwareDestroy() {

    }

    @Override
    public void __RunOpMode() {
        this.getRobotCore().chassis.getLocalizer().update();
        this.getRobotCore().setCurrentPosition(ElysiumAutonomousSettings.BLUE_AUTO_START_POSE);
        //Sample First
        this.sampledPosition = m_Sampler.sample(ALLIANCE_TYPE);
        FtcDashboard.getInstance().sendImage(this.m_Sampler.LastSampledFrame);
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
            //do as many stones as possible
            for(int i=0; i<2; i++) {
                this.runToStoneAndPlaceOnFoundation(getStoneOrder[i],i);
            }
        }
        {
            //not much time left, left's grab the foundation and park
            if(!this.grabFoundationAndPark()){
                return;
            }
        }
    }

    public boolean runToStoneAndPlaceOnFoundation(int stoneNumber, int numRuns){
        RobotPose2D currentPosition = this.getRobotCore().getCurrentPosition();
        double grabStoneStartSpeed = 0.1;
        RobotPoint2D stonePosition = allStonePositions.get(stoneNumber - 1);
        stonePosition.Y -= 3 * (stoneNumber + 1);
        if(currentPosition.X >= 0){
            //gotta pass the bridge first
            this.setLeftClawToRest();

            RobotPoint2D firstPoint = ElysiumAutoBase.getBuildingZoneFurtherFromBridgePoint(ALLIANCE_TYPE,PARK_POSITION);
            firstPoint.Y += 0;
            RobotPoint2D secondPoint = ElysiumAutoBase.getLoadingZoneFurtherFromBridgePoint(ALLIANCE_TYPE,PARK_POSITION);
            secondPoint.Y += 0;

            RobotPoint2D thirdPoint = new RobotPoint2D(stonePosition);
            thirdPoint.Y += 20;

            Trajectory trajectory = this.m_Core.chassis.trajectoryBuilder()
                    .lineTo(getRoadRunnerPos(firstPoint),new ConstantInterpolator(Math.toRadians(-180)))
                    .lineTo(getRoadRunnerPos(secondPoint),new ConstantInterpolator(Math.toRadians(-180)))
                    .addMarker(()->{
                        setLeftClawToPrepareGrab();
                        return Unit.INSTANCE;
                    })
                    .lineTo(getRoadRunnerPos(thirdPoint),new ConstantInterpolator(Math.toRadians(-180)))
                    .lineTo(getRoadRunnerPos(stonePosition),new ConstantInterpolator(Math.toRadians(-180)))
                    .build();
            this.m_Core.chassis.followTrajectorySync(trajectory);
            if(!this.opModeIsActive()){
                return false;
            }
            this.calibratePositionUsingDistanceSensor();
            if(!this.opModeIsActive()){
                return false;
            }
        }else{
            //go to the stone.
            RobotPoint2D pointAwayFromWall = ElysiumAutonomousSettings.BLUE_AUTO_START_POSE;
            pointAwayFromWall.Y -= 35;

            RobotPoint2D stonePrepPosition = new RobotPoint2D(stonePosition);
            stonePrepPosition.Y += 10;

            Trajectory trajectory = this.m_Core.chassis.trajectoryBuilder()
                    .lineTo(getRoadRunnerPos(pointAwayFromWall),new ConstantInterpolator(Math.toRadians(-180)))
                    .addMarker(()->{
                        setLeftClawToPrepareGrab();
                        return Unit.INSTANCE;
                    })
                    .lineTo(getRoadRunnerPos(stonePrepPosition),new ConstantInterpolator(Math.toRadians(-180)))
                    .lineTo(getRoadRunnerPos(stonePosition),new ConstantInterpolator(Math.toRadians(-180)))
                    .build();
            this.m_Core.chassis.followTrajectorySync(trajectory);
            if(!this.opModeIsActive()){
                return false;
            }
        }
        {
            //grab stone
            this.grabLeftClaw();
            if(!this.opModeIsActive()){
                return false;
            }
            this.calibratePositionUsingDistanceSensor();
        }
        {
            //finished grabbing stone, now we should head to the foundation.
            RobotPoint2D firstPoint = ElysiumAutoBase.getLoadingZoneFurtherFromBridgePoint(ALLIANCE_TYPE,PARK_POSITION);
            firstPoint.Y += 0;

            RobotPoint2D secondPoint = ElysiumAutoBase.getLoadingZoneNextToBridgePoint(ALLIANCE_TYPE,PARK_POSITION);
            secondPoint.Y += 0;

            RobotPoint2D thirdPoint = ElysiumAutoBase.getBuildingZoneFurtherFromBridgePoint(ALLIANCE_TYPE,PARK_POSITION);
            thirdPoint.Y += 0;

            RobotPoint2D fourthPoint = lastDropStonePos;

            Trajectory trajectory = this.m_Core.chassis.trajectoryBuilder()
                    .lineTo(getRoadRunnerPos(firstPoint),new ConstantInterpolator(Math.toRadians(-180)))
                    .lineTo(getRoadRunnerPos(secondPoint),new ConstantInterpolator(Math.toRadians(-180)))
                    .lineTo(getRoadRunnerPos(thirdPoint),new ConstantInterpolator(Math.toRadians(-180)))
                    .addMarker(()->{
                        setLeftClawToPrepareDrop();
                        return Unit.INSTANCE;
                    })
                    .lineTo(getRoadRunnerPos(fourthPoint),new ConstantInterpolator(Math.toRadians(-180)))
                    .build();
            this.m_Core.chassis.followTrajectorySync(trajectory);
            if(!this.opModeIsActive()){
                return false;
            }
            lastDropStonePos.X -= 25;
            this.calibratePositionUsingDistanceSensor();
        }
        {
            //release stone from hand
            super.setLeftClawToDropStone();
            super.setLeftClawToRest();
        }
        {
            RobotPose2D currentPose = this.getRobotCore().getCurrentPosition();
            currentPose.Y += 5;
            this.getRobotCore().setCurrentPosition(currentPose);
        }
        return true;
    }
    public boolean grabFoundationAndPark(){
        RobotPose2D currentPosition = this.getRobotCore().getCurrentPosition();
        if(currentPosition.X <= 0){
            //we are not in the building zone, let's go to the building zone.
            this.setLeftClawToRest();
            RobotPoint2D firstPoint = ElysiumAutoBase.getLoadingZoneFurtherFromBridgePoint(ALLIANCE_TYPE,PARK_POSITION);
            firstPoint.Y += 0;

            RobotPoint2D secondPoint = ElysiumAutoBase.getLoadingZoneNextToBridgePoint(ALLIANCE_TYPE,PARK_POSITION);
            secondPoint.Y += 0;

            RobotPoint2D thirdPoint = ElysiumAutoBase.getBuildingZoneFurtherFromBridgePoint(ALLIANCE_TYPE,PARK_POSITION);
            thirdPoint.Y += 0;

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
            this.getRobotCore().stackerSubSystem.stackerSlide.replaceTask(new TargetPosTask(null,ElysiumSettings.STACKER_ABOVE_FOUNDATION_POS,ElysiumAutonomousSettings.STACKER_SLIDE_SPEED));
            this.getRobotCore().stackerSubSystem.setDoorState(ElysiumStacker.Stacker_Door_State.RELEASED);

            RobotPoint2D grabFoundationPos = ElysiumAutoBase.grabFoundationPosition_BLUE;
            RobotPoint2D grabFoundationPrepPos = new RobotPoint2D(grabFoundationPos);
            grabFoundationPrepPos.Y += 20;
            this.m_Core.chassis.followTrajectory(
                    this.getRobotCore().chassis.trajectoryBuilder()
                            .lineTo(getRoadRunnerPos(grabFoundationPrepPos),new ConstantInterpolator(Math.toRadians(-180)))
                            .build()
            );
            while(this.getRobotCore().chassis.isBusy() && this.opModeIsActive()){
                this.getRobotCore().stackerSubSystem.stackerSlide.updateStatus();
                this.getRobotCore().chassis.update();
            }
            if(!this.opModeIsActive()){
                return false;
            }

            this.m_Core.chassis.turn(Math.toRadians(-90));
            while(this.getRobotCore().chassis.isBusy() && this.opModeIsActive()){
                this.getRobotCore().stackerSubSystem.stackerSlide.updateStatus();
                this.getRobotCore().chassis.update();
            }

            this.m_Core.chassis.followTrajectory(
                    this.getRobotCore().chassis.trajectoryBuilder()
                            .lineTo(getRoadRunnerPos(grabFoundationPos),new ConstantInterpolator(Math.toRadians(90)))
                            .build()
            );
            while(this.getRobotCore().chassis.isBusy() && this.opModeIsActive()){
                this.getRobotCore().stackerSubSystem.stackerSlide.updateStatus();
                this.getRobotCore().chassis.update();
            }
            if(!this.opModeIsActive()){
                return false;
            }

            //set puller to grab foundation
            this.grabFoundation(ElysiumAutonomousSettings.STACKER_SLIDE_SPEED);
            this.calibratePositionUsingDistanceSensor();
        }
        {
            //Now the foundation is in our hands, let's pull it.
            RobotPoint2D firstPoint = new RobotPoint2D(ElysiumAutoBase.grabFoundationPosition_BLUE);
            firstPoint.X -= 10;
            firstPoint.Y = SkyStoneCoordinates.BLUE_BUILDING_ZONE_FIELD_EXTREME_POINT.Y - (SkyStoneCoordinates.TILE_FLOOR_WIDTH + ElysiumSettings.FOUNDATION_HOOK_DISTANCE_FROM_CENTER - 35);
            RobotPoint2D secondPoint = new RobotPoint2D(firstPoint);
            secondPoint.X = SkyStoneCoordinates.BLUE_BUILDING_ZONE_FIELD_EXTREME_POINT.X - (SkyStoneCoordinates.FOUNDATION_WIDTH + ElysiumSettings.PHYSICAL_CENTER_TO_BACK) + 10;

            this.getRobotCore().chassis.followTrajectorySync(
                    this.getRobotCore().chassis.trajectoryBuilder()
                            .lineTo(getRoadRunnerPos(firstPoint),new ConstantInterpolator(Math.toRadians(90)))
                            .build()
            );
            if(!this.opModeIsActive()){
                return false;
            }

            this.getRobotCore().chassis.turnSync(Math.toRadians(90));
            this.getRobotCore().stackerSubSystem.stackerSlide.replaceTask(new TargetPosTask(null,ElysiumSettings.STACKER_ABOVE_FOUNDATION_POS,ElysiumAutonomousSettings.STACKER_SLIDE_SPEED));

            this.getRobotCore().chassis.followTrajectory(
                    this.getRobotCore().chassis.trajectoryBuilder()
                            .lineTo(getRoadRunnerPos(secondPoint),new ConstantInterpolator(Math.toRadians(-180)))
                            .build()
            );
            while(this.m_Core.chassis.isBusy()){
                if(this.isStopRequested()){
                    return false;
                }
                this.m_Core.chassis.update();
                this.getRobotCore().stackerSubSystem.stackerSlide.updateStatus();
            }
            //calibrate our position first
            RobotPose2D foundationFinishedPose = new RobotPose2D(secondPoint,-180);
            RobotPose2D currentPose = this.getRobotCore().getCurrentPosition();
            foundationFinishedPose.setRotationZ(currentPose.getRotationZ());
            foundationFinishedPose.X = SkyStoneCoordinates.BLUE_BUILDING_ZONE_FIELD_EXTREME_POINT.X - (SkyStoneCoordinates.FOUNDATION_WIDTH + ElysiumSettings.PHYSICAL_CENTER_TO_BACK);
            foundationFinishedPose.Y -= 15;
            this.useSensorToCalibrateNegativeYPosition(foundationFinishedPose,-ElysiumSettings.LOCALIZATION_RIGHTDISTSENSOR_POS.Y,this.getRobotCore().RightSensor.Sensor,XYPlaneCalculations.normalizeDeg(foundationFinishedPose.getRotationZ() + 180));
            this.getRobotCore().setCurrentPosition(foundationFinishedPose);
        }
        {
            //foundation pulled, let's get our hands out of the foundation
            RobotPoint2D leaveFoundationPoint = new RobotPoint2D(this.getRobotCore().getCurrentPosition());
            leaveFoundationPoint.X -= 20;

            RobotPoint2D firstPoint = ElysiumAutoBase.getBuildingZoneFurtherFromBridgePoint(ALLIANCE_TYPE,PARK_POSITION);
            firstPoint.Y += 5;

            RobotPoint2D secondPoint = SkyStoneCoordinates.getParkPosition(ALLIANCE_TYPE,PARK_POSITION);
            secondPoint.Y += 0;

            this.getRobotCore().stackerSubSystem.stackerSlide.replaceTask(new TargetPosTask(null,ElysiumSettings.STACKER_ABOVE_FOUNDATION_POS,ElysiumAutonomousSettings.STACKER_SLIDE_SPEED));

            Trajectory trajectory = this.getRobotCore().chassis.trajectoryBuilder()
                    .lineTo(getRoadRunnerPos(leaveFoundationPoint),new ConstantInterpolator(-180))
                    .addMarker(()->{
                        this.getRobotCore().stackerSubSystem.stackerSlide.replaceTask(new TargetPosTask(null,ElysiumSettings.STACKER_SLIDE_MIN_POS,ElysiumAutonomousSettings.STACKER_SLIDE_SPEED));
                        return Unit.INSTANCE;
                    })
                    .lineTo(getRoadRunnerPos(firstPoint),new ConstantInterpolator(Math.toRadians(-180)))
                    .lineTo(getRoadRunnerPos(secondPoint), new ConstantInterpolator(Math.toRadians(-180)))
                    .build();
            this.m_Core.chassis.followTrajectory(trajectory);
            while(this.m_Core.chassis.isBusy() && this.opModeIsActive()){
                this.getRobotCore().stackerSubSystem.stackerSlide.updateStatus();
                this.m_Core.chassis.update();
            }
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
