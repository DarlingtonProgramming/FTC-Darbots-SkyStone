package org.darbots.darbotsftclib.utilities.log_utilities;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.darbots.darbotsftclib.libcore_4_5_0Pre.runtime.GlobalUtil;

@TeleOp(group = "DarbotsLib-Utilities", name = "DeleteAllLogs")
public class DeleteLogs extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("LogDeleter","Press Start to Delete ALL Logs");
        telemetry.update();
        waitForStart();
        if(opModeIsActive()){
            telemetry.addData("LogDeleter","Deleting...");
            telemetry.update();
            GlobalUtil.deleteAllLogs();
            telemetry.addData("LogDeleter","Finished");
            telemetry.update();
        }
        while(opModeIsActive()){

        }
    }
}
