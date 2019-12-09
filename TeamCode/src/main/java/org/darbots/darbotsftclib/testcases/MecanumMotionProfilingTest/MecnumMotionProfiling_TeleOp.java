package org.darbots.darbotsftclib.testcases.MecanumMotionProfilingTest;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.darbots.darbotsftclib.libcore.OpModes.DarbotsBasicOpMode;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPose2D;
import org.darbots.darbotsftclib.libcore.motion_planning.followers.TrajectoryFollower;
import org.darbots.darbotsftclib.libcore.motion_planning.paths.LinePath;
import org.darbots.darbotsftclib.libcore.motion_planning.trajectories.SimpleTrajectoryGenerator;
import org.darbots.darbotsftclib.libcore.templates.chassis_related.MotionSystemConstraints;
import org.darbots.darbotsftclib.libcore.templates.motion_planning.RobotPath;
import org.darbots.darbotsftclib.libcore.templates.motion_planning.RobotTrajectory;
import org.darbots.darbotsftclib.testcases.common.TestMecanumCore;

import java.util.InvalidPropertiesFormatException;

@TeleOp(group = "DarbotsLib-TestCases", name = "MecanumMotionProfilingTest")
public class MecnumMotionProfiling_TeleOp extends DarbotsBasicOpMode<TestMecanumCore> {
    public static final double CONST_TEST_DISTANCE = 30;
    public static final double CONST_TEST_STARTSPEED_NORMALIZED = 0;
    public static final double CONST_TEST_CRUISESPEED_NORMALIZED = 0.7;
    public static final double CONST_TEST_ENDSPEED_NORMALIZED = 0;
    public static final double CONST_TRAJECTORY_RESOLUTION = 0.02;
    public static final double CONST_MAX_ACCELERATION_NORMALIZED = 0.2;
    public static final double CONST_TEST_PREFERRED_NEW_ANGLE = 0;
    public static final double CONST_MAX_ANGULAR_ACCELERATION_NORMALIZED = 0.2;

    private TestMecanumCore m_Core;
    @Override
    public TestMecanumCore getRobotCore() {
        return null;
    }

    @Override
    public void hardwareInitialize() {
        this.m_Core = new TestMecanumCore(this.hardwareMap,"MecanumMotionProfilingTest.log");
    }

    @Override
    public void hardwareDestroy() {

    }

    @Override
    public void RunThisOpMode() {
        while(this.opModeIsActive()){
            this.m_Core.updateStatus();

            if(gamepad1.x){
                this.m_Core.getChassis().getPositionTracker().resetRelativeOffset();
            }
            telemetry.addData("Note","to reset offset Position, simply press x on gamepad 1");

            if(!this.m_Core.getChassis().isBusy()){
                if(Math.abs(gamepad1.left_stick_x) > 0.15 || Math.abs(gamepad1.left_stick_y) > 0.15){
                    double mappedX = -gamepad1.left_stick_y;
                    double mappedY = -gamepad1.left_stick_x;
                    double mappedDistance= Math.sqrt(Math.pow(mappedX,2) + Math.pow(mappedY,2));
                    double mappedFactor = CONST_TEST_DISTANCE / mappedDistance;
                    double pathX = mappedX * mappedFactor;
                    double pathY = mappedY * mappedFactor;
                    LinePath pathToGo = new LinePath(pathX, pathY);
                    double startSpeed = CONST_TEST_STARTSPEED_NORMALIZED * this.m_Core.getChassis().calculateMaxLinearSpeedInCMPerSec();
                    double cruiseSpeed = CONST_TEST_CRUISESPEED_NORMALIZED * this.m_Core.getChassis().calculateMaxLinearSpeedInCMPerSec();
                    double endSpeed = CONST_TEST_ENDSPEED_NORMALIZED * this.m_Core.getChassis().calculateMaxLinearSpeedInCMPerSec();
                    MotionSystemConstraints constraints = this.m_Core.getChassis().getMotionSystemConstraints(
                            this.m_Core.getChassis().calculateMaxLinearSpeedInCMPerSec() * CONST_MAX_ACCELERATION_NORMALIZED,
                            0,
                            this.m_Core.getChassis().calculateMaxAngularSpeedInDegPerSec() * CONST_MAX_ANGULAR_ACCELERATION_NORMALIZED,
                            0
                    );

                    RobotTrajectory trajectoryToGo = SimpleTrajectoryGenerator.generateTrajectory(CONST_TRAJECTORY_RESOLUTION, constraints, pathToGo, startSpeed, cruiseSpeed, endSpeed,CONST_TEST_PREFERRED_NEW_ANGLE);
                    TrajectoryFollower follower = new TrajectoryFollower(trajectoryToGo);
                    this.m_Core.getChassis().addTask(follower);
                }
            }

            this.m_Core.updateTelemetry();
            telemetry.update();
        }
    }
}
