package org.firstinspires.ftc.teamcode.david_cao.Gen2_Lindel_Code;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(group = "4100", name = "LindelTeleOp-NoLimit-Gen2.1")
public class LindelTeleOp_NoLimit extends LindelTeleOp {
    @Override
    public void hardwareInitialize(){
        super.hardwareInitialize();
        this.getRobotCore().getLinearSlide().setMinPos(-1000);
        this.getRobotCore().getLinearSlide().setMaxPos(1000);
    }
}
