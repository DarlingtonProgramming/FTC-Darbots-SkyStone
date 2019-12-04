package org.firstinspires.ftc.teamcode.david_cao.generation1_lindel_code;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.darbots.darbotsftclib.libcore.OpModes.DarbotsBasicOpMode;
import org.darbots.darbotsftclib.libcore.calculations.dimentionalcalculation.Robot3DPositionIndicator;
import org.darbots.darbotsftclib.libcore.sensors.cameras.RobotOnPhoneCamera;
import org.darbots.darbotsftclib.season_specific.skystone.darbots_vuforia_skystone_detection.DarbotsSkyStoneDifferentiation;
import org.firstinspires.ftc.teamcode.robot_common.Robot4100Common;

@Autonomous(name = "4100Gen1Auto-RedBuildSiteComplex",group="4100")
public class Robot4100Generation2_RedBuildSiteComplex extends DarbotsBasicOpMode<Robot4100Generation2_LindelCore> {
    private Robot4100Generation2_LindelCore m_RobotCore;
    private DarbotsSkyStoneDifferentiation m_Navigation;
    private int ScanResult = 0;
    private RobotOnPhoneCamera Camera;
    @Override
    public Robot4100Generation2_LindelCore getRobotCore() {
        return m_RobotCore;
    }

    @Override
    public void hardwareInitialize() {
        this.m_RobotCore = new Robot4100Generation2_LindelCore(this.hardwareMap);
        Camera = new RobotOnPhoneCamera(this, Robot4100Generation2_Settings.AUTONOMOUS_TENSORFLOW_PREVIEW, RobotOnPhoneCamera.PhoneCameraDirection.Back, Robot4100Common.VUFORIA_LICENSE);
        Robot3DPositionIndicator CameraPosition = new Robot3DPositionIndicator(Robot4100Generation2_Settings.AUTONOMOUS_CAMERAPOSONPHONE);
        this.m_Navigation = new DarbotsSkyStoneDifferentiation(CameraPosition,Camera);
    }

    @Override
    public void hardwareDestroy() {
        this.m_RobotCore.terminate();
    }

    @Override
    public void RunThisOpMode() {
        this.getRobotCore().getChassis().setGyroGuidedDriveEnabled(true);
        this.getRobotCore().getChassis().updateGyroGuidedPublicStartingAngle();

        this.m_RobotCore.getChassis().replaceTask(
                this.m_RobotCore.getChassis().getFixedZDistanceTask(
                        -40,
                        0.4
                )
        );
        if(!waitForDrive()){
            return;
        }

        this.m_RobotCore.getChassis().replaceTask(this.m_RobotCore.getChassis().getFixedXDistanceTask(
                -30,
                0.5
        ));
        if(!waitForDrive())
            return;

        this.m_RobotCore.getChassis().replaceTask(
                this.m_RobotCore.getChassis().getFixedZDistanceTask(
                        -40,
                        0.3
                )
        );
        if(!waitForDrive()){
            return;
        }

        this.m_RobotCore.setDragServoToDrag(true);
        sleep(500);
        this.m_RobotCore.getChassis().replaceTask(
                this.m_RobotCore.getChassis().getFixedZDistanceTask(
                        95,
                        0.5
                )
        );
        if(!waitForDrive()){
            return;
        }

        this.m_RobotCore.setDragServoToDrag(false);
        sleep(300);

        this.m_RobotCore.getChassis().replaceTask(
                this.m_RobotCore.getChassis().getFixedXDistanceTask(
                        90,
                        0.5
                )
        );
        if(!waitForDrive()){
            return;
        }

        this.m_RobotCore.getChassis().replaceTask(
                this.m_RobotCore.getChassis().getFixedZDistanceTask(
                        -45,
                        0.5
                )
        );
        if(!waitForDrive()){
            return;
        }

        this.m_RobotCore.getChassis().replaceTask(
                this.m_RobotCore.getChassis().getFixedXDistanceTask(
                        -50,
                        0.4
                )
        );

        this.m_RobotCore.getChassis().addTask(
                this.m_RobotCore.getChassis().getFixedXDistanceTask(
                        20,
                        0.3
                )
        );
        if(!waitForDrive()){
            return;
        }

        this.m_RobotCore.getChassis().addTask(
                this.m_RobotCore.getChassis().getFixedZDistanceTask(
                        -5,
                        0.25
                )
        );
        if(!waitForDrive()){
            return;
        }



        this.m_RobotCore.getChassis().addTask(this.m_RobotCore.getChassis().getFixedTurnTask(
                -90,
                0.5
        ));
        if(!waitForDrive()){
            return;
        }



        this.m_RobotCore.getChassis().replaceTask(
                this.m_RobotCore.getChassis().getFixedZDistanceTask(
                        110,
                        0.5
                )
        );
        if(!waitForDrive()){
            return;
        }
        this.m_RobotCore.getChassis().replaceTask(
                this.m_RobotCore.getChassis().getFixedXDistanceTask(
                        -20,
                        0.4
                )
        );
        if(!waitForDrive()){
            return;
        }
        this.m_Navigation.setActivated(true);
        Camera.setFlashlightEnabled(true);
        double firstScanExtraDistance = 0;
        int[] stepResult = {1,2,3};
        for(int i=0; i<3; i++){

            //if(i<2){ //0 or 1
            this.getRobotCore().getChassis().replaceTask(this.getRobotCore().getChassis().getFixedXDistanceTask(
                    -5,
                    0.15
            ));
            this.getRobotCore().getChassis().addTask(this.getRobotCore().getChassis().getFixedXDistanceTask(
                    5,
                    0.15
            ));
            if(!waitForDrive()){
                return;
            }
            //}


            boolean foundStone = false;
            if(this.m_Navigation.getDarbotsRobotAxisStonePosition() != null || i == 2){
                foundStone = true;
            }
            if(foundStone){
                this.ScanResult = stepResult[i];
                break;
            }
            firstScanExtraDistance += Robot4100Generation2_Settings.AUTONOMOUS_LENGTH_FOR_EACH_STONE;
            this.getRobotCore().getChassis().replaceTask(this.getRobotCore().getChassis().getFixedZDistanceTask(
                    Robot4100Generation2_Settings.AUTONOMOUS_LENGTH_FOR_EACH_STONE,
                    0.25
            ));
            if(!waitForDrive()){
                return;
            }
        }
        Camera.setFlashlightEnabled(false);
        this.m_Navigation.setActivated(false);
        double firstScanZOffset = 0;
        double firstScanXOffset = 0;
        Robot3DPositionIndicator firstScanStonePosition = this.m_Navigation.getDarbotsRobotAxisStonePosition();
        if(firstScanStonePosition != null){
            firstScanZOffset = -firstScanStonePosition.getZ();
            firstScanXOffset = -firstScanStonePosition.getX();
        }
        firstScanExtraDistance -= Robot4100Generation2_Settings.AUTONOMOUS_DISTANCE_BETWEEN_PHONE_AND_STONEGRABBER;
        telemetry.addData("ZOffset",firstScanZOffset);
        telemetry.addData("XOffset",firstScanXOffset);
        telemetry.addData("distanceToMove",firstScanZOffset + Robot4100Generation2_Settings.AUTONOMOUS_DISTANCE_BETWEEN_PHONE_AND_STONEGRABBER);
        telemetry.update();
        this.getRobotCore().getChassis().replaceTask(this.getRobotCore().getChassis().getFixedZDistanceTask(
                firstScanZOffset + Robot4100Generation2_Settings.AUTONOMOUS_DISTANCE_BETWEEN_PHONE_AND_STONEGRABBER,
                0.15
        ));
        if(!waitForDrive()){
            return;
        }
        this.getRobotCore().getChassis().replaceTask(this.getRobotCore().getChassis().getFixedXDistanceTask(
                50,//firstScanXOffset + 10,
                0.2
        ));
        if(!waitForDrive()){
            return;
        }

        this.getRobotCore().setAutonomousDragStoneServoRightToDrag(true);
        sleep(400);

        this.getRobotCore().getChassis().replaceTask(this.getRobotCore().getChassis().getFixedXDistanceTask(
                -35,
                0.5
        ));
        if(!waitForDrive()){
            return;
        }

        this.getRobotCore().getChassis().replaceTask(this.getRobotCore().getChassis().getFixedTurnTask(
                -90,
                0.5
        ));

        if(!waitForDrive()){
            return;
        }

        this.getRobotCore().getChassis().replaceTask(this.getRobotCore().getChassis().getFixedXDistanceTask(
                125 + firstScanExtraDistance,
                1.0
        ));
        if(!waitForDrive()){
            return;
        }

        this.getRobotCore().setAutonomousDragStoneServoRightToDrag(false);
        sleep(400);

        this.getRobotCore().getChassis().replaceTask(this.getRobotCore().getChassis().getFixedXDistanceTask(
                -65,
                0.5
        ));
        if(!waitForDrive()){
            return;
        }

    }
}