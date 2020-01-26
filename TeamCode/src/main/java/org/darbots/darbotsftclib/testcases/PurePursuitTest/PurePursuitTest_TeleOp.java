package org.darbots.darbotsftclib.testcases.PurePursuitTest;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.darbots.darbotsftclib.libcore.OpModes.DarbotsBasicOpMode;
import org.darbots.darbotsftclib.libcore.purepursuit.followers.PurePursuitPathFollower;
import org.darbots.darbotsftclib.libcore.purepursuit.utils.PurePursuitWayPoint;
import org.darbots.darbotsftclib.libcore.templates.odometry.RobotActive2DPositionTracker;
import org.darbots.darbotsftclib.testcases.common.TestMecanumCore;

import java.util.ArrayList;

@TeleOp(group = "DarbotsLib-TestCases", name = "MecanumPurePursuitTest")
@Disabled
public class PurePursuitTest_TeleOp extends DarbotsBasicOpMode<TestMecanumCore> {
    public static final double CONST_MAX_ACCEL_NORMALIZED = 0.2;
    public static final double CONST_TEST_STARTSPEED_NORMALIZED = 0.2;
    public static final double CONST_TEST_CRUISESPEED_NORMALIZED = 0.8;
    public static final double CONST_TEST_ENDSPEED_NORMALIZED = 0.0;
    public static final double CONST_TEST_ANGLESPEED_NORMALIZED = 0.4;
    public static final double CONST_TEST_FOLLOW_RADIUS = 25;
    public static final double CONST_TEST_PREFERRED_ANGLE = 0;

    private TestMecanumCore m_Core;
    private ArrayList<PurePursuitWayPoint> m_Waypoints;

    @Override
    public TestMecanumCore getRobotCore() {
        return this.m_Core;
    }

    public void initializeWaypoints(){
        this.m_Waypoints = new ArrayList<>();
        this.m_Waypoints.add(new PurePursuitWayPoint(120,0));
        this.m_Waypoints.add(new PurePursuitWayPoint(120,-120));
        this.m_Waypoints.add(new PurePursuitWayPoint(240,0));

    }

    @Override
    public void hardwareInitialize() {
        this.m_Core = new TestMecanumCore(this.hardwareMap,"MecanumMotionProfilingTest.log");
        ((RobotActive2DPositionTracker) this.m_Core.getChassis().getPositionTracker()).setGyroProvider(this.m_Core.getGyro());

        this.initializeWaypoints();
    }

    @Override
    public void hardwareDestroy() {
        this.m_Core = null;
    }

    @Override
    public void RunThisOpMode() {
        double ACCELERATION = this.CONST_MAX_ACCEL_NORMALIZED * this.getRobotCore().getChassis().calculateMaxLinearSpeedInCMPerSec(this.CONST_TEST_PREFERRED_ANGLE);
        double STARTSPEED = this.CONST_TEST_STARTSPEED_NORMALIZED * this.getRobotCore().getChassis().calculateMaxLinearSpeedInCMPerSec(this.CONST_TEST_PREFERRED_ANGLE);
        double CRUISESPEED = this.CONST_TEST_CRUISESPEED_NORMALIZED * this.getRobotCore().getChassis().calculateMaxLinearSpeedInCMPerSec(this.CONST_TEST_PREFERRED_ANGLE);
        double ENDSPEED = this.CONST_TEST_ENDSPEED_NORMALIZED * this.getRobotCore().getChassis().calculateMaxLinearSpeedInCMPerSec(this.CONST_TEST_PREFERRED_ANGLE);
        double ANGLESPEED = this.CONST_TEST_ANGLESPEED_NORMALIZED * this.getRobotCore().getChassis().calculateMaxAngularSpeedInDegPerSec();

        PurePursuitPathFollower pathFollower = new PurePursuitPathFollower(
                this.m_Waypoints,
                ACCELERATION,
                STARTSPEED,
                CRUISESPEED,
                ENDSPEED,
                ANGLESPEED,
                CONST_TEST_FOLLOW_RADIUS,
                CONST_TEST_PREFERRED_ANGLE
        );

        this.getRobotCore().getChassis().addTask(pathFollower);
        while(this.getRobotCore().getChassis().isBusy()){
            if(this.isStopRequested()){
                return;
            }
            this.getRobotCore().updateStatus();
            this.getRobotCore().updateTelemetry();
            telemetry.update();
        }
    }
}
