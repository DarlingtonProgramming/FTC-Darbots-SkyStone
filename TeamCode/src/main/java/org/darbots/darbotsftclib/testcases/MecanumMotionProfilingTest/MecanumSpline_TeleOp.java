package org.darbots.darbotsftclib.testcases.MecanumMotionProfilingTest;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.darbots.darbotsftclib.libcore.OpModes.DarbotsBasicOpMode;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPoint2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPose2D;
import org.darbots.darbotsftclib.libcore.motion_planning.followers.TrajectoryFollower;
import org.darbots.darbotsftclib.libcore.motion_planning.paths.LinePath;
import org.darbots.darbotsftclib.libcore.motion_planning.paths.SplinePath.SplinePath;
import org.darbots.darbotsftclib.libcore.motion_planning.trajectories.SimpleTrajectoryGenerator;
import org.darbots.darbotsftclib.libcore.templates.chassis_related.MotionSystemConstraints;
import org.darbots.darbotsftclib.libcore.templates.motion_planning.RobotPath;
import org.darbots.darbotsftclib.libcore.templates.motion_planning.RobotTrajectory;
import org.darbots.darbotsftclib.testcases.common.TestMecanumCore;

import java.util.ArrayList;
import java.util.InvalidPropertiesFormatException;

@TeleOp(group = "DarbotsLib-TestCases", name = "MecanumSplineTest")
public class MecanumSpline_TeleOp extends DarbotsBasicOpMode<TestMecanumCore> {
    public static final double CONST_TEST_STARTSPEED_NORMALIZED = 0;
    public static final double CONST_TEST_CRUISESPEED_NORMALIZED = 0.7;
    public static final double CONST_TEST_ENDSPEED_NORMALIZED = 0;
    public static final double CONST_TRAJECTORY_RESOLUTION = 0.02;
    public static final double CONST_PATH_INTEGRATION_RESOLUTION = 0.001;
    public static final double CONST_MAX_ACCELERATION_NORMALIZED = 0.2;
    public static final double CONST_TEST_PREFERRED_NEW_ANGLE = 0;
    public static final double CONST_MAX_ANGULAR_ACCELERATION_NORMALIZED = 0.2;

    private TestMecanumCore m_Core;
    private ArrayList<RobotPoint2D> m_SplinePoints;
    private RobotTrajectory m_SplineTrajectory;
    @Override
    public TestMecanumCore getRobotCore() {
        return this.m_Core;
    }

    public void initializeSplinePoints(){
        this.m_SplinePoints = new ArrayList<>();

    }

    @Override
    public void hardwareInitialize() {
        telemetry.addData("Status","Initializing RobotCore");
        telemetry.update();
        this.m_Core = new TestMecanumCore(this.hardwareMap,"MecanumMotionProfilingTest.log");
        telemetry.addData("Status","Initializing Spline Points");
        telemetry.update();
        this.initializeSplinePoints();
        telemetry.addData("Status","Initializing Spline Path");
        telemetry.update();
        SplinePath mSplinePath = new SplinePath(this.m_SplinePoints,CONST_PATH_INTEGRATION_RESOLUTION);

        telemetry.addData("Status","Forming Spline Trajectory");
        telemetry.update();
        double maxLinearSpeed = this.m_Core.getChassis().calculateMaxLinearSpeedInCMPerSec();
        double startSpeed = CONST_TEST_STARTSPEED_NORMALIZED * maxLinearSpeed;
        double cruiseSpeed = CONST_TEST_CRUISESPEED_NORMALIZED * maxLinearSpeed;
        double endSpeed = CONST_TEST_ENDSPEED_NORMALIZED * maxLinearSpeed;
        double maxAccel = CONST_MAX_ACCELERATION_NORMALIZED * maxLinearSpeed;
        double maxAngularSpeed = this.m_Core.getChassis().calculateMaxAngularSpeedInDegPerSec();
        double maxAngularAccel = CONST_MAX_ANGULAR_ACCELERATION_NORMALIZED * maxAngularSpeed;
        MotionSystemConstraints constraints = this.m_Core.getChassis().getMotionSystemConstraints(maxAccel,0,maxAngularAccel,0);
        this.m_SplineTrajectory = SimpleTrajectoryGenerator.generateTrajectory(
                CONST_TRAJECTORY_RESOLUTION,
                constraints,
                mSplinePath,
                startSpeed,
                cruiseSpeed,
                endSpeed,
                CONST_TEST_PREFERRED_NEW_ANGLE
        );
    }

    @Override
    public void hardwareDestroy() {
        this.m_SplineTrajectory = null;
        this.m_SplinePoints = null;
        this.m_Core = null;
    }

    @Override
    public void RunThisOpMode() {
        this.m_Core.getChassis().addTask(new TrajectoryFollower(this.m_SplineTrajectory));
        waitForDrive_WithTelemetry();
    }
}
