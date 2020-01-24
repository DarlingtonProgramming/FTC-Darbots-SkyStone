package org.firstinspires.ftc.teamcode.david_cao.Gen2_Lindel_Code;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.darbots.darbotsftclib.libcore.OpModes.DarbotsBasicOpMode;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPoint2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPose2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.XYPlaneCalculations;
import org.darbots.darbotsftclib.libcore.tasks.servo_tasks.motor_powered_servo_tasks.TargetPosTask;
import org.darbots.darbotsftclib.libcore.templates.DarbotsComboKey;

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
}
