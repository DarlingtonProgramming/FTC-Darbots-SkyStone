package org.darbots.darbotsftclib.testcases.DarbotsVuforiaTargetTest;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.darbots.darbotsftclib.libcore_4_5_0Pre.OpModes.DarbotsBasicOpMode;
import org.darbots.darbotsftclib.libcore_4_5_0Pre.calculations.dimentional_calculation.RobotPose3D;
import org.darbots.darbotsftclib.libcore_4_5_0Pre.sensors.cameras.RobotOnPhoneCamera;
import org.darbots.darbotsftclib.libcore_4_5_0Pre.templates.RobotCore;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot_common.Robot4100Common;

@TeleOp(group = "DarbotsLib-TestCases", name = "DifferentialStone-LibTest")
@Disabled
public class DarbotsSkyStoneVuforiaTargetConceptValidation extends DarbotsBasicOpMode {
    DarbotsSkyStoneNavigation_TestDifferenciation Detector;
    @Override
    public RobotCore getRobotCore() {
        return null;
    }

    @Override
    public void hardwareInitialize() {
        RobotOnPhoneCamera myCamera = new RobotOnPhoneCamera(this,true, RobotOnPhoneCamera.PhoneCameraDirection.Back, Robot4100Common.VUFORIA_LICENSE);
        RobotPose3D PhonePosition = new RobotPose3D(
                0,
                0,
                0,
                0,
                90,
                -90
        );
        Detector = new DarbotsSkyStoneNavigation_TestDifferenciation(PhonePosition,myCamera);
        Detector.setActivated(true);
    }

    @Override
    public void hardwareDestroy() {
        Detector.setActivated(false);
    }

    @Override
    public void RunThisOpMode() {
        while(this.opModeIsActive()){
            RobotPose3D OriginalPose = Detector.getDarbotsRobotAxisOriginalStonePosition();
            RobotPose3D DarbotsPose = Detector.getDarbotsRobotAxisDarbotsStonePosition();
            if(OriginalPose != null) {
                Telemetry.Line OriginalLine = telemetry.addLine("Original Pose");
                OriginalLine.addData("X", OriginalPose.X);
                OriginalLine.addData("Y", OriginalPose.Y);
                OriginalLine.addData("Z", OriginalPose.Z);
            }
            if(DarbotsPose != null) {
                Telemetry.Line DarbotsLine = telemetry.addLine("Darbots Pose");
                DarbotsLine.addData("X", DarbotsPose.X);
                DarbotsLine.addData("Y", DarbotsPose.Y);
                DarbotsLine.addData("Z", DarbotsPose.Z);
                telemetry.update();
            }
        }
    }
}
