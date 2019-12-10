package org.darbots.darbotsftclib.testcases.PurePursuitTest;

import org.darbots.darbotsftclib.libcore.OpModes.DarbotsBasicOpMode;
import org.darbots.darbotsftclib.libcore.purepursuit.followers.PurePursuitPathFollower;
import org.darbots.darbotsftclib.libcore.purepursuit.utils.PurePursuitWayPoint;
import org.darbots.darbotsftclib.testcases.common.TestMecanumCore;

import java.util.ArrayList;

public class PurePursuitTest_TeleOp extends DarbotsBasicOpMode<TestMecanumCore> {
    public static final double CONST_MAX_ACCEL_NORMALIZED = 0.1;
    public static final double CONST_TEST_STARTSPEED_NORMALIZED = 0.0;
    public static final double CONST_TEST_CRUISESPEED_NORMALIZED = 0.5;
    public static final double CONST_TEST_ENDSPEED_NORMALIZED = 0.0;
    public static final double CONST_TEST_ANGLESPEED_NORMALIZED = 0.3;
    public static final double CONST_TEST_FOLLOW_RADIUS = 20;
    public static final double CONST_TEST_PREFERRED_ANGLE = 0;

    private TestMecanumCore m_Core;
    private ArrayList<PurePursuitWayPoint> m_Waypoints;

    @Override
    public TestMecanumCore getRobotCore() {
        return this.m_Core;
    }

    public void initializeWaypoints(){
        this.m_Waypoints = new ArrayList<>();
    }

    @Override
    public void hardwareInitialize() {
        this.m_Core = new TestMecanumCore(this.hardwareMap,"MecanumMotionProfilingTest.log");
    }

    @Override
    public void hardwareDestroy() {
        this.m_Core = null;
    }

    @Override
    public void RunThisOpMode() {
        double ACCELERATION = this.CONST_MAX_ACCEL_NORMALIZED * this.getRobotCore().getChassis().calculateMaxLinearSpeedInCMPerSec();
        double STARTSPEED = this.CONST_TEST_STARTSPEED_NORMALIZED * this.getRobotCore().getChassis().calculateMaxLinearSpeedInCMPerSec();
        double CRUISESPEED = this.CONST_TEST_CRUISESPEED_NORMALIZED * this.getRobotCore().getChassis().calculateMaxLinearSpeedInCMPerSec();
        double ENDSPEED = this.CONST_TEST_ENDSPEED_NORMALIZED * this.getRobotCore().getChassis().calculateMaxLinearSpeedInCMPerSec();
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
        if(!waitForDrive()){
            return;
        }
    }
}
