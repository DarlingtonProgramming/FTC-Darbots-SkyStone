package org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.RoadRunner.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.XYPlaneCalculations;
import org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.RoadRunner.drive.mecanum.ElysiumRoadRunnerBase;
import org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.RoadRunner.drive.mecanum.ElysiumRoadRunnerChassis;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
public class StraightTest extends LinearOpMode {
    public static double DISTANCE = 45 / XYPlaneCalculations.INCH_PER_CM;

    @Override
    public void runOpMode() throws InterruptedException {
        ElysiumRoadRunnerBase drive = new ElysiumRoadRunnerChassis(hardwareMap);

        Trajectory trajectory = drive.trajectoryBuilder()
                .forward(DISTANCE)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySync(trajectory);
    }
}
