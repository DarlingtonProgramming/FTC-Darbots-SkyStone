package org.firstinspires.ftc.teamcode.david_cao.Gen2_Lindel_Code;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.darbots.darbotsftclib.game_specific.AllianceType;
import org.darbots.darbotsftclib.libcore_4_5_0Pre.calculations.dimentional_calculation.RobotPoint2D;
import org.darbots.darbotsftclib.libcore_4_5_0Pre.calculations.dimentional_calculation.RobotPose2D;
import org.darbots.darbotsftclib.libcore_4_5_0Pre.runtime.MovementUtil;
import org.darbots.darbotsftclib.season_specific.skystone.ParkPosition;
import org.darbots.darbotsftclib.season_specific.skystone.SkyStoneCoordinates;

@Autonomous(group = "4100", name = "Lindel-Red-LoadingZone-Eagle")
public class LindelRedParkFromLoadingZone_Eagle extends LindelAutoBase {
    public static final ParkPosition PARK_POSITION = ParkPosition.NEXT_TO_NEUTRAL_BRIDGE;
    public static final AllianceType ALLIANCE = AllianceType.RED;
    @Override
    public void __init() {
        //Initial Position - Right side of the robot lined up with the end of 2nd Tile from the audience wall
        RobotPose2D initPose = new RobotPose2D(SkyStoneCoordinates.RED_LOADING_ZONE_FIELD_EXTREME_POINT,90);
        initPose.X += 119.38 + 3 + LindelSettings.PHYSICAL_WIDTH;
        initPose.Y += LindelSettings.LENGTH_FROM_CENTER_TO_BACK;
        this.getRobotCore().PosTracker.setCurrentPosition(initPose);
    }

    @Override
    public void __destroy() {

    }

    @Override
    public void __updateTelemetry() {

    }

    @Override
    public void RunThisOpMode() {
        RobotPoint2D parkPosition = SkyStoneCoordinates.getParkPosition(ALLIANCE,PARK_POSITION);
        RobotPose2D robotPointToPark = this.getRobotPose(new RobotPose2D(parkPosition,90));
        double preferredAngle = this.getRobotCore().getChassis().getPreferredAngle(90);
        delay(25);
        this.getRobotCore().getChassis().replaceTask(MovementUtil.getGoToPointTask(robotPointToPark.X,robotPointToPark.Y,0,0.5*this.getRobotCore().getChassis().calculateMaxLinearSpeedInCMPerSec(),0,preferredAngle));
        if(!this.waitForDrive_WithTelemetry()){
            return;
        }
    }
}
