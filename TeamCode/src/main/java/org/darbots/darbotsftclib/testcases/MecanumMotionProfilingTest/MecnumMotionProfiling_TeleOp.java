package org.darbots.darbotsftclib.testcases.MecanumMotionProfilingTest;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.darbots.darbotsftclib.libcore.OpModes.DarbotsBasicOpMode;
import org.darbots.darbotsftclib.libcore.motion_planning.followers.TrajectoryFollower;
import org.darbots.darbotsftclib.libcore.motion_planning.paths.LinePath;
import org.darbots.darbotsftclib.libcore.motion_planning.trajectories.SimpleTrajectory;
import org.darbots.darbotsftclib.libcore.motion_planning.trajectories.SimpleTrajectoryGenerator;
import org.darbots.darbotsftclib.libcore.templates.chassis_related.MotionSystemConstraints;
import org.darbots.darbotsftclib.libcore.templates.motion_planning.RobotPath;
import org.darbots.darbotsftclib.libcore.templates.motion_planning.RobotTrajectory;
import org.darbots.darbotsftclib.libcore.templates.servo_related.ContinuousRotationServoType;
import org.darbots.darbotsftclib.testcases.MecanumPosTrackerTest.MecanumPosTrackerTest_Core;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.InvalidPropertiesFormatException;

@TeleOp(group = "DarbotsLib-TestCases", name = "MecanumMotionProfilingTest")
public class MecnumMotionProfiling_TeleOp extends DarbotsBasicOpMode<MecanumMotionProfilingTest_Core> {
    public static final double CONST_TEST_DISTANCE = 30;
    public static final double CONST_TEST_DISTANCE_SIDELENGTH = CONST_TEST_DISTANCE / Math.sqrt(2);
    public static final double CONST_TEST_STARTSPEED_NORMALIZED = 0;
    public static final double CONST_TEST_CRUISESPEED_NORMALIZED = 0.7;
    public static final double CONST_TEST_ENDSPEED_NORMALIZED = 0;
    public static final double CONST_TRAJECTORY_RESOLUTION = 0.02;
    public static final double CONST_MAX_ACCELERATION_NORMALIZED = 0.05;
    public static final double CONST_TEST_PREFERRED_NEW_ANGLE = 0;

    private MecanumMotionProfilingTest_Core m_Core;
    private RobotTrajectory m_LTTrack, m_RTTrack, m_LBTrack, m_RBTrack, m_PXTrack, m_PYTrack, m_NXTrack, m_NYTrack;

    @Override
    public MecanumMotionProfilingTest_Core getRobotCore() {
        return null;
    }

    @Override
    public void hardwareInitialize() {
        this.m_Core = new MecanumMotionProfilingTest_Core(this.hardwareMap);
        MotionSystemConstraints constraints = this.m_Core.getChassis().getMotionSystemConstraints(
                this.m_Core.getChassis().calculateMaxLinearSpeedCombinationsInCMPerSec() * CONST_MAX_ACCELERATION_NORMALIZED,
                0
        );
        RobotPath LTPath, RTPath, LBPath, RBPath, PXPath, PYPath, NXPath, NYPath;
        try{
            telemetry.addData("Status","Generating LTPath");
            telemetry.update();
            LTPath = new LinePath(CONST_TEST_DISTANCE_SIDELENGTH,CONST_TEST_DISTANCE_SIDELENGTH);
            telemetry.addData("Status","Generating RTPath");
            telemetry.update();
            RTPath = new LinePath(CONST_TEST_DISTANCE_SIDELENGTH,-CONST_TEST_DISTANCE_SIDELENGTH);
            telemetry.addData("Status","Generating LBPath");
            telemetry.update();
            LBPath = new LinePath(-CONST_TEST_DISTANCE_SIDELENGTH,CONST_TEST_DISTANCE_SIDELENGTH);
            telemetry.addData("Status","Generating RBPath");
            telemetry.update();
            RBPath = new LinePath(-CONST_TEST_DISTANCE_SIDELENGTH,-CONST_TEST_DISTANCE_SIDELENGTH);
            telemetry.addData("Status","Generating PXPath");
            telemetry.update();
            PXPath = new LinePath(CONST_TEST_DISTANCE,0);
            telemetry.addData("Status","Generating PYPath");
            telemetry.update();
            PYPath = new LinePath(0,CONST_TEST_DISTANCE);
            telemetry.addData("Status","Generating NXPath");
            telemetry.update();
            NXPath = new LinePath(-CONST_TEST_DISTANCE,0);
            telemetry.addData("Status","Generating NYPath");
            telemetry.update();
            NYPath = new LinePath(0,-CONST_TEST_DISTANCE);
        }catch(InvalidPropertiesFormatException e){
            e.printStackTrace();
            return;
        }
        double startSpeed = CONST_TEST_STARTSPEED_NORMALIZED * this.m_Core.getChassis().calculateMaxLinearSpeedCombinationsInCMPerSec();
        double cruiseSpeed = CONST_TEST_CRUISESPEED_NORMALIZED * this.m_Core.getChassis().calculateMaxLinearSpeedCombinationsInCMPerSec();
        double endSpeed = CONST_TEST_ENDSPEED_NORMALIZED * this.m_Core.getChassis().calculateMaxLinearSpeedCombinationsInCMPerSec();

        telemetry.addData("Status","Generating LT Trajectory");
        telemetry.update();
        this.m_LTTrack = SimpleTrajectoryGenerator.generateTrajectory(CONST_TRAJECTORY_RESOLUTION, constraints, LTPath, startSpeed, cruiseSpeed, endSpeed);
        telemetry.addData("Status","Generating RT Trajectory");
        telemetry.update();
        this.m_RTTrack = SimpleTrajectoryGenerator.generateTrajectory(CONST_TRAJECTORY_RESOLUTION, constraints, RTPath, startSpeed, cruiseSpeed, endSpeed);
        telemetry.addData("Status","Generating LB Trajectory");
        telemetry.update();
        this.m_LBTrack = SimpleTrajectoryGenerator.generateTrajectory(CONST_TRAJECTORY_RESOLUTION, constraints, LBPath, startSpeed, cruiseSpeed, endSpeed);
        telemetry.addData("Status","Generating RB Trajectory");
        telemetry.update();
        this.m_RBTrack = SimpleTrajectoryGenerator.generateTrajectory(CONST_TRAJECTORY_RESOLUTION, constraints, RBPath, startSpeed, cruiseSpeed, endSpeed);
        telemetry.addData("Status","Generating PX Trajectory");
        telemetry.update();
        this.m_PXTrack = SimpleTrajectoryGenerator.generateTrajectory(CONST_TRAJECTORY_RESOLUTION, constraints, PXPath, startSpeed, cruiseSpeed, endSpeed);
        telemetry.addData("Status","Generating PY Trajectory");
        telemetry.update();
        this.m_PYTrack = SimpleTrajectoryGenerator.generateTrajectory(CONST_TRAJECTORY_RESOLUTION, constraints, PYPath, startSpeed, cruiseSpeed, endSpeed);
        telemetry.addData("Status","Generating NX Trajectory");
        telemetry.update();
        this.m_NXTrack = SimpleTrajectoryGenerator.generateTrajectory(CONST_TRAJECTORY_RESOLUTION, constraints, NXPath, startSpeed, cruiseSpeed, endSpeed);
        telemetry.addData("Status","Generating NY Trajectory");
        telemetry.update();
        this.m_NYTrack = SimpleTrajectoryGenerator.generateTrajectory(CONST_TRAJECTORY_RESOLUTION, constraints, NYPath, startSpeed, cruiseSpeed, endSpeed);
    }

    @Override
    public void hardwareDestroy() {
        this.m_LTTrack = null;
        this.m_RTTrack = null;
        this.m_LBTrack = null;
        this.m_RBTrack = null;
        this.m_PXTrack = null;
        this.m_PYTrack = null;
        this.m_NXTrack = null;
        this.m_NYTrack = null;
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
                if(gamepad1.left_stick_x > 0.2 || gamepad1.left_stick_y > 0.2){
                    RobotTrajectory trajectoryToGo = null;
                    double mappedX = -gamepad1.left_stick_y;
                    double mappedY = -gamepad1.left_stick_x;
                    if(Math.abs(mappedX) >= Math.abs(mappedY) / 2.0 && Math.abs(mappedX) <= Math.abs(mappedY) * 2.0){
                        if(mappedX > 0 && mappedY > 0){
                            trajectoryToGo = m_LTTrack;
                        }else if(mappedX > 0 && mappedY < 0){
                            trajectoryToGo = m_RTTrack;
                        }else if(mappedX < 0 && mappedY > 0){
                            trajectoryToGo = m_LBTrack;
                        }else{ //mappedX <= 0 && mappedY <= 0
                            trajectoryToGo = m_RBTrack;
                        }
                    }else{
                        if(Math.abs(mappedX) > Math.abs(mappedY)){
                            if(mappedX > 0){
                                trajectoryToGo = m_PXTrack;
                            }else{
                                trajectoryToGo = m_NXTrack;
                            }
                        }else{
                            if(mappedY > 0){
                                trajectoryToGo = m_PYTrack;
                            }else{
                                trajectoryToGo = m_NXTrack;
                            }
                        }
                    }
                    TrajectoryFollower follower = new TrajectoryFollower(trajectoryToGo,CONST_TEST_PREFERRED_NEW_ANGLE);
                    this.m_Core.getChassis().addTask(follower);
                }
            }

            this.m_Core.updateTelemetry();
            telemetry.update();
        }
    }
}
