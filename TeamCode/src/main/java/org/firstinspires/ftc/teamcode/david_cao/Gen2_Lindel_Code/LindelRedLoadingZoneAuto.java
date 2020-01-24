package org.firstinspires.ftc.teamcode.david_cao.Gen2_Lindel_Code;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPose2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPose3D;
import org.darbots.darbotsftclib.libcore.runtime.MovementUtil;
import org.darbots.darbotsftclib.libcore.sensors.cameras.RobotOnPhoneCamera;
import org.darbots.darbotsftclib.season_specific.skystone.SkyStoneCoordinates;
import org.darbots.darbotsftclib.season_specific.skystone.darbots_vuforia_skystone_detection.DarbotsSkyStoneDifferentiation;
import org.darbots.darbotsftclib.season_specific.skystone.tfod_detection.SkyStoneStoneDifferentiation;
import org.firstinspires.ftc.teamcode.robot_common.Robot4100Common;

@Autonomous(group = "4100", name = "Lindel-Red-LoadingZone")
public class LindelRedLoadingZoneAuto extends LindelAutoBase {
    protected DarbotsSkyStoneDifferentiation m_VuforiaSampler;
    protected RobotOnPhoneCamera m_Camera;

    @Override
    public void RunThisOpMode() {
        //Initial Position - Right side of the robot lined up with the end of 2nd Tile from the audience wall
        RobotPose2D initPose = new RobotPose2D(SkyStoneCoordinates.RED_LOADING_ZONE_FIELD_EXTREME_POINT,90);
        initPose.X += 119.38 - LindelSettings.PHYSICAL_WIDTH / 2;
        initPose.Y += LindelSettings.LENGTH_FROM_CENTER_TO_BACK;
        this.getRobotCore().PosTracker.setCurrentPosition(initPose);
        this.getRobotCore().getChassis().replaceTask(MovementUtil.getGoToPointTask(40,5,0,this.maxAutoSpeed,0,0));
        if(!waitForDrive_WithTelemetry()){
            return;
        }
        while(this.opModeIsActive()){
            sleep(20);
        }
    }

    @Override
    public void __init() {
        m_Camera = new RobotOnPhoneCamera(this,true, RobotOnPhoneCamera.PhoneCameraDirection.Back, Robot4100Common.VUFORIA_LICENSE);
        m_VuforiaSampler = new DarbotsSkyStoneDifferentiation(new RobotPose3D(0,0,0,0,-90,0),this.m_Camera);
        RobotOnPhoneCamera.setFlashlightEnabled(LindelSettings.AUTO_SAMPLE_LIGHT);
        m_VuforiaSampler.setActivated(true);
    }

    @Override
    public void __destroy() {
        RobotOnPhoneCamera.setFlashlightEnabled(false);
    }
}
