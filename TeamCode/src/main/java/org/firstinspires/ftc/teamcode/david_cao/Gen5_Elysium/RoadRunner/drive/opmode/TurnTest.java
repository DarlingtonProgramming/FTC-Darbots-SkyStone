package org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.RoadRunner.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.RoadRunner.drive.mecanum.ElysiumRoadRunnerBase;
import org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.RoadRunner.drive.mecanum.ElysiumRoadRunnerChassis;

/*
 * This is a simple routine to test turning capabilities.
 */
@Config
@Autonomous(group = "drive")
public class TurnTest extends LinearOpMode {
    public static double ANGLE = 90; // deg

    @Override
    public void runOpMode() throws InterruptedException {
        ElysiumRoadRunnerBase drive = new ElysiumRoadRunnerChassis(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        drive.turnSync(Math.toRadians(ANGLE));
    }
}
