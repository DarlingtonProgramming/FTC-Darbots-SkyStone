package org.firstinspires.ftc.teamcode.david_cao.generation1_lindel_code;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.darbots.darbotsftclib.libcore.OpModes.DarbotsBasicOpMode;
import org.darbots.darbotsftclib.libcore.runtime.GlobalUtil;

@Autonomous(group = "4100", name = "4100Gen1-MovementSquare")
@Disabled
public class Robot4100Generation2_AutoMovementSquare extends DarbotsBasicOpMode<Robot4100Generation2_LindelCore> {
    private Robot4100Generation2_LindelCore m_RobotCore;
    private String m_ChassisStatus;
    public String getChassisStatus(){
        return this.m_ChassisStatus;
    }
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
        this.m_RobotCore = null;
    }

    @Override
    public void RunThisOpMode() {
        waitForStart();
        if(!this.opModeIsActive()){
            return;
        }
        if(GlobalUtil.getGyro() != null) {
            this.m_RobotCore.getChassis().setGyroGuidedDriveEnabled(true);
            this.m_RobotCore.getChassis().updateGyroGuidedPublicStartingAngle();
        }
        while(this.opModeIsActive()){
            this.m_RobotCore.getChassis().replaceTask(this.m_RobotCore.getChassis().getFixedZDistanceTask(
                    30,
                    0.5
            ));
            if(!waitForDrive())
                return;
            this.m_RobotCore.getChassis().replaceTask(this.m_RobotCore.getChassis().getFixedXDistanceTask(
                    30,
                    0.5
            ));
            if(!waitForDrive())
                return;
            this.m_RobotCore.getChassis().replaceTask(this.m_RobotCore.getChassis().getFixedZDistanceTask(
                    -30,
                    0.5
            ));
            if(!waitForDrive())
                return;
            this.m_RobotCore.getChassis().replaceTask(this.m_RobotCore.getChassis().getFixedXDistanceTask(
                    -30,
                    0.5
            ));
            if(!waitForDrive())
                return;
        }
    }
}
