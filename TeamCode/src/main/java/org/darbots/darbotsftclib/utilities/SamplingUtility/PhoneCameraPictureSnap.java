package org.darbots.darbotsftclib.utilities.SamplingUtility;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.darbots.darbotsftclib.libcore.integratedfunctions.FTCFileIO;
import org.darbots.darbotsftclib.libcore.runtime.GlobalUtil;
import org.darbots.darbotsftclib.libcore.sensors.cameras.RobotOnPhoneCamera;
import org.firstinspires.ftc.teamcode.robot_common.Robot4100Common;

@TeleOp(group = "DarbotsLib-Utilities", name = "PhoneCameraPictureSnapUtility")
public class PhoneCameraPictureSnap extends LinearOpMode {
    public static RobotOnPhoneCamera.PhoneCameraDirection CameraDirection = RobotOnPhoneCamera.PhoneCameraDirection.Back;
    public static boolean enableFlashLight = false;
    public static String vuforiaKey = Robot4100Common.VUFORIA_LICENSE;
    public static String filename = "DarbotsSnapshot.jpg";

    @Override
    public void runOpMode() throws InterruptedException {
        RobotOnPhoneCamera camera = new RobotOnPhoneCamera(this,true,CameraDirection,vuforiaKey);
        RobotOnPhoneCamera.setFlashlightEnabled(enableFlashLight);

        {
            TelemetryPacket packet = new TelemetryPacket();
            GlobalUtil.addTelmetryLine(this.telemetry, packet, "Info", "Welcome to picture snap Op Mode, press start to take a snapshot");
            telemetry.update();
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }

        waitForStart();
        if(this.opModeIsActive()){
            Bitmap frame = camera.getFrame();
            FTCFileIO.writeBitmapFile(FTCFileIO.getSnapshotFolderFile(filename),frame,100, Bitmap.CompressFormat.JPEG);
            {
                TelemetryPacket packet = new TelemetryPacket();
                GlobalUtil.addTelmetryLine(this.telemetry,packet,"Info", "Picture saved to /FIRST/Snapshots/" + filename);
                telemetry.update();
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
                FtcDashboard.getInstance().sendImage(frame);
            }
            while(this.opModeIsActive()){
                sleep(20);
            }
        }
    }
}
