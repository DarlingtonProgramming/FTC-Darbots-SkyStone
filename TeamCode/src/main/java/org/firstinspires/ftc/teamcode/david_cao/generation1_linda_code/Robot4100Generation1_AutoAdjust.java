package org.firstinspires.ftc.teamcode.david_cao.generation1_linda_code;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.darbots.darbotsftclib.libcore.OpModes.DarbotsBasicOpMode;
import org.darbots.darbotsftclib.libcore.runtime.GlobalUtil;

@Autonomous(group = "4100", name = "4100Gen1-Autoadjust")
public class Robot4100Generation1_AutoAdjust extends DarbotsBasicOpMode<Robot4100Generation1_LindaCore> {
    private Robot4100Generation1_LindaCore m_RobotCore;
    private String m_ChassisStatus;
    public String getChassisStatus(){
        return this.m_ChassisStatus;
    }
    @Override
    public Robot4100Generation1_LindaCore getRobotCore() {
        return m_RobotCore;
    }

    @Override
    public void hardwareInitialize() {
        this.m_RobotCore = new Robot4100Generation1_LindaCore(this.hardwareMap);
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

        this.m_RobotCore.getChassis().replaceTask(this.m_RobotCore.getChassis().getFixedXDistanceTask(
                -30,
                0.5
        ));
        if(!waitForDrive())
            return;


    }
}
