package org.firstinspires.ftc.teamcode.david_cao.Gen2_Lindel_Code;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(group = "4100", name = "LindelTeleOp-NoMemory-Gen2.1")
public class LindelTeleOp_NoMemory extends LindelTeleOp {
    @Override
    public void RunThisOpMode() {
        this.getRobotCore().setDragServoToDrag(false);
        while(this.opModeIsActive()) {
            driveControl();
            foundationGraberControl();
            resetSlideControl();
            slideControl();
            grabberControl();
            grabberRotControl();
            intakeControl();
            capstoneControl();
            orientServoControl();
            ElevatorUpControl();
            ElevatorDownControl();
            StackUpControl();

            this.updateStatus();

            this.getRobotCore().updateTelemetry();
            telemetry.update();
        }
    }
}
