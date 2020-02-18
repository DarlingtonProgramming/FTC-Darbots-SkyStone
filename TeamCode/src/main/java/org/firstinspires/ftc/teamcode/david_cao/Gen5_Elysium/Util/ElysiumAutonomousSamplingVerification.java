package org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.Util;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.darbots.darbotsftclib.game_specific.AllianceType;
import org.darbots.darbotsftclib.libcore.OpModes.DarbotsBasicOpMode;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPose2D;
import org.darbots.darbotsftclib.libcore.integratedfunctions.FTCFileIO;
import org.darbots.darbotsftclib.libcore.runtime.GlobalUtil;
import org.darbots.darbotsftclib.libcore.sensors.cameras.RobotOnPhoneCamera;
import org.darbots.darbotsftclib.libcore.templates.RobotCore;
import org.darbots.darbotsftclib.season_specific.skystone.SkyStonePosition;
import org.darbots.darbotsftclib.season_specific.skystone.darbots_pixel_skystone_detection.DarbotsPixelSkyStoneSampler;
import org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.Autonomous.ElysiumAutoSampler;
import org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.ElysiumAutoCore;
import org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.Elysium_Settings.ElysiumAutonomousSettings;
import org.firstinspires.ftc.teamcode.robot_common.Robot4100Common;

@Config
public class ElysiumAutonomousSamplingVerification extends LinearOpMode {
    public static boolean sampleBlue = false;
    private RobotOnPhoneCamera m_Camera;
    private ElysiumAutoSampler m_Sampler;
    private SkyStonePosition sampledPosition;

    @Override
    public void runOpMode() throws InterruptedException {
        this.m_Camera = new RobotOnPhoneCamera(this,true, RobotOnPhoneCamera.PhoneCameraDirection.Back, Robot4100Common.VUFORIA_LICENSE);
        this.m_Sampler = new ElysiumAutoSampler(this.m_Camera);
        {
            TelemetryPacket packet = new TelemetryPacket();
            GlobalUtil.addTelmetryLine(this.telemetry, packet, "Info", "Welcome to Autonomous Sampling Verification Op Mode, press start to take a snapshot");
            telemetry.update();
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
        waitForStart();
        if(this.opModeIsActive()){
            AllianceType sampleAlliance = sampleBlue ? AllianceType.BLUE : AllianceType.RED;
            this.sampledPosition = m_Sampler.sample(sampleAlliance);
            {
                TelemetryPacket packet = new TelemetryPacket();
                GlobalUtil.addTelmetryLine(this.telemetry,packet,"Info","Sampling Finished");
                GlobalUtil.addTelmetryLine(this.telemetry, packet, "SkyStone Position", sampledPosition.name());
                telemetry.update();
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
                FtcDashboard.getInstance().sendImage(m_Sampler.getLastSampledFrame());
            }
            while(this.opModeIsActive()){
                sleep(20);
            }
        }
    }
}
