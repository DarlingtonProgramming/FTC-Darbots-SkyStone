package org.darbots.darbotsftclib.testcases.DarbotsVuforiaTargetTest;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.darbots.darbotsftclib.libcore.OpModes.DarbotsBasicOpMode;
import org.darbots.darbotsftclib.libcore.calculations.dimentionalcalculation.Robot3DPositionIndicator;
import org.darbots.darbotsftclib.libcore.sensors.cameras.RobotOnPhoneCamera;
import org.darbots.darbotsftclib.libcore.templates.RobotCore;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot_common.Robot4100Common;

@TeleOp(group = "DarbotsLib-TestCases", name = "DifferentialStone-LibTest")
public class DarbotsSkyStoneVuforiaTargetConceptValidation extends DarbotsBasicOpMode {
    DarbotsSkyStoneNavigation_TestDifferenciation Detector;
    @Override
    public RobotCore getRobotCore() {
        return null;
    }

    @Override
    public void hardwareInitialize() {
        RobotOnPhoneCamera myCamera = new RobotOnPhoneCamera(this,true, RobotOnPhoneCamera.PhoneCameraDirection.Back, Robot4100Common.VUFORIA_LICENSE);
        Robot3DPositionIndicator PhonePosition = new Robot3DPositionIndicator(
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
            Robot3DPositionIndicator OriginalPose = Detector.getDarbotsRobotAxisOriginalStonePosition();
            Robot3DPositionIndicator DarbotsPose = Detector.getDarbotsRobotAxisDarbotsStonePosition();
            if(OriginalPose != null) {
                Telemetry.Line OriginalLine = telemetry.addLine("Original Pose");
                OriginalLine.addData("X", OriginalPose.getX());
                OriginalLine.addData("Y", OriginalPose.getY());
                OriginalLine.addData("Z", OriginalPose.getZ());
            }
            if(DarbotsPose != null) {
                Telemetry.Line DarbotsLine = telemetry.addLine("Darbots Pose");
                DarbotsLine.addData("X", DarbotsPose.getX());
                DarbotsLine.addData("Y", DarbotsPose.getY());
                DarbotsLine.addData("Z", DarbotsPose.getZ());
                telemetry.update();
            }
        }
    }
}
