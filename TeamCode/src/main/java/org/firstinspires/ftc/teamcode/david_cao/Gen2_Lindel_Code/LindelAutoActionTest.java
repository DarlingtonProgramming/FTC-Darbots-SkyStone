package org.firstinspires.ftc.teamcode.david_cao.Gen2_Lindel_Code;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(group = "4100", name = "LindelAutoTest")
public class LindelAutoActionTest extends LindelAutoBase {

    @Override
    public void RunThisOpMode() {
        this.getRobotCore().setDragServoToDrag(false);
        while(this.opModeIsActive()){
            if(gamepad1.x){
                this.startSuckStones();
            }else if(gamepad1.y){
                this.stopSuckStones();
            }else if(gamepad1.dpad_up){
                this.depositStoneToFoundation();
            }
            this.getRobotCore().updateTelemetry();
            this.updateStatus();
            telemetry.update();
        }
    }


    @Override
    public void __init() {

    }

    @Override
    public void __destroy() {

    }

    @Override
    public void __updateTelemetry() {

    }
}
