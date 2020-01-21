package org.firstinspires.ftc.teamcode.david_cao.Gen4_SwanSilver_Code;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.darbots.darbotsftclib.libcore.tasks.chassis_tasks.RobotMotionSystemTeleOpTask;

@TeleOp(group = "4100", name = "4100Gen4-TeleOp-UNLIMITED")
@Disabled
public class SwanSilverTeleOp_UNLIMITED extends SwanSilverTeleOp {

    @Override
    public void hardwareInitialize() {
        super.hardwareInitialize();
        super.getRobotCore().Slide.setMaxPos(1000);
        super.getRobotCore().Slide.setMinPos(-1000);
    }
}
