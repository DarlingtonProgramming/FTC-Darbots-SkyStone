package org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.RoadRunner.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.XYPlaneCalculations;
import org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.RoadRunner.drive.mecanum.ElysiumRoadRunnerBase;
import org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.RoadRunner.drive.mecanum.ElysiumRoadRunnerChassis;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ElysiumRoadRunnerBase drive = new ElysiumRoadRunnerChassis(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(30 / XYPlaneCalculations.INCH_PER_CM, 30 / XYPlaneCalculations.INCH_PER_CM, 0))
                        .build()
        );

        sleep(2000);

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .reverse()
                        .splineTo(new Pose2d(0, 0, 0))
                        .build()
        );
    }
}
