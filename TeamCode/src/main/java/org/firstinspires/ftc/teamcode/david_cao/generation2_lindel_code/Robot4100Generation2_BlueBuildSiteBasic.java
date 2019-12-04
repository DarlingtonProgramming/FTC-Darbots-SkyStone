package org.firstinspires.ftc.teamcode.david_cao.generation2_lindel_code;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.darbots.darbotsftclib.libcore.OpModes.DarbotsBasicOpMode;

@Autonomous(name = "4100Gen1Auto-BlueBuildSiteBasic",group="4100")
public class Robot4100Generation2_BlueBuildSiteBasic extends DarbotsBasicOpMode<Robot4100Generation2_LindelCore> {
    private Robot4100Generation2_LindelCore m_RobotCore;
    private float m_OldAng;
    @Override
    public Robot4100Generation2_LindelCore getRobotCore() {
        return m_RobotCore;
    }

    @Override
    public void hardwareInitialize() {
        this.m_RobotCore = new Robot4100Generation2_LindelCore(this.hardwareMap);
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
                25,
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
                        -90,
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
                        40,
                        0.4
                )
        );

        this.m_RobotCore.getChassis().addTask(
                this.m_RobotCore.getChassis().getFixedXDistanceTask(
                        -20,
                        0.3
                )
        );
        if(!waitForDrive()){
            return;
        }



        this.m_RobotCore.getChassis().replaceTask(
                this.m_RobotCore.getChassis().getFixedXDistanceTask(
                        -60,
                        0.3
                )
        );
        if(!waitForDrive()){
            return;
        }
    }
}
