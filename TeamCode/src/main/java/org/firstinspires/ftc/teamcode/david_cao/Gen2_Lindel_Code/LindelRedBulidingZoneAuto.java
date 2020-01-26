package org.firstinspires.ftc.teamcode.david_cao.Gen2_Lindel_Code;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.darbots.darbotsftclib.libcore_4_5_0Pre.runtime.MovementUtil;

@Autonomous(group = "4100", name = "Lindel-Red-BuildingZone")
public class LindelRedBulidingZoneAuto extends LindelAutoBase {
    @Override
    public void __init() {

    }

    @Override
    public void __destroy() {

    }

    @Override
    public void __updateTelemetry() {

    }

    @Override
    public void RunThisOpMode() {
        this.getRobotCore().getChassis().replaceTask(MovementUtil.getGoToPointTask(-40,40,0,0.5 * this.getRobotCore().getChassis().calculateMaxLinearXSpeedInCMPerSec(),0,0));
        if(!this.waitForDrive_WithTelemetry()){
            return;
        }
        this.getRobotCore().getChassis().replaceTask(MovementUtil.getGoToPointTask(-40,0,0,0.5 * this.getRobotCore().getChassis().calculateMaxLinearXSpeedInCMPerSec(),0,0));
        if(!this.waitForDrive_WithTelemetry()){
            return;
        }
        this.getRobotCore().setDragServoToDrag(true);
        this.getRobotCore().getChassis().replaceTask(MovementUtil.getGoToPointTask(90,0,0,0.5 * this.getRobotCore().getChassis().calculateMaxLinearXSpeedInCMPerSec(),0,0));
        if(!this.waitForDrive_WithTelemetry()){
            return;
        }
        this.getRobotCore().setDragServoToDrag(false);
        this.getRobotCore().getChassis().replaceTask(MovementUtil.getGoToPointTask(0,-140,0,0.5 * this.getRobotCore().getChassis().calculateMaxLinearXSpeedInCMPerSec(),0,0));
        if(!this.waitForDrive_WithTelemetry()){
            return;
        }
    }
}
