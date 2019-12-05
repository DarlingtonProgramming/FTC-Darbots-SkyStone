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
    public static final double CONST_TEST_DISTANCE_SIDELENGTH = CONST_TEST_DISTANCE / Math.sqrt(2);
    public static final double CONST_TEST_STARTSPEED_NORMALIZED = 0;
    public static final double CONST_TEST_CRUISESPEED_NORMALIZED = 0.7;
    public static final double CONST_TEST_ENDSPEED_NORMALIZED = 0;
    public static final double CONST_TRAJECTORY_RESOLUTION = 0.02;
    public static final double CONST_MAX_ACCELERATION_NORMALIZED = 0.1;
    public static final double CONST_TEST_PREFERRED_NEW_ANGLE = 0;

    private RobotPath LTPath, RTPath, LBPath, RBPath, PXPath, PYPath, NXPath, NYPath;
    private TestMecanumCore m_Core;
    @Override
    public TestMecanumCore getRobotCore() {
        return null;
    }

    @Override
    public void hardwareInitialize() {
        this.m_Core = new TestMecanumCore(this.hardwareMap,"MecanumMotionProfilingTest.log");
        MotionSystemConstraints constraints = this.m_Core.getChassis().getMotionSystemConstraints(
                this.m_Core.getChassis().calculateMaxLinearSpeedInCMPerSec() * CONST_MAX_ACCELERATION_NORMALIZED,
                0
        );

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
                if(Math.abs(gamepad1.left_stick_x) > 0.2 || Math.abs(gamepad1.left_stick_y) > 0.2){
                    RobotPath pathToGo = null;
                    double mappedX = -gamepad1.left_stick_y;
                    double mappedY = -gamepad1.left_stick_x;
                    if(Math.abs(mappedX) >= Math.abs(mappedY) / 2.0 && Math.abs(mappedX) <= Math.abs(mappedY) * 2.0){
                        if(mappedX > 0 && mappedY > 0){
                            pathToGo = LTPath;
                        }else if(mappedX > 0 && mappedY < 0){
                            pathToGo = RTPath;
                        }else if(mappedX < 0 && mappedY > 0){
                            pathToGo = LBPath;
                        }else{ //mappedX <= 0 && mappedY <= 0
                            pathToGo = RBPath;
                        }
                    }else{
                        if(Math.abs(mappedX) > Math.abs(mappedY)){
                            if(mappedX > 0){
                                pathToGo = PXPath;
                            }else{
                                pathToGo = NXPath;
                            }
                        }else{
                            if(mappedY > 0){
                                pathToGo = PYPath;
                            }else{
                                pathToGo = NXPath;
                            }
                        }
                    }
                    double startSpeed = CONST_TEST_STARTSPEED_NORMALIZED * this.m_Core.getChassis().calculateMaxLinearSpeedInCMPerSec();
                    double cruiseSpeed = CONST_TEST_CRUISESPEED_NORMALIZED * this.m_Core.getChassis().calculateMaxLinearSpeedInCMPerSec();
                    double endSpeed = CONST_TEST_ENDSPEED_NORMALIZED * this.m_Core.getChassis().calculateMaxLinearSpeedInCMPerSec();
                    MotionSystemConstraints constraints = this.m_Core.getChassis().getMotionSystemConstraints(
                            this.m_Core.getChassis().calculateMaxLinearSpeedInCMPerSec() * CONST_MAX_ACCELERATION_NORMALIZED,
                            0
                    );

                    RobotTrajectory trajectoryToGo = SimpleTrajectoryGenerator.generateTrajectory(CONST_TRAJECTORY_RESOLUTION, constraints, pathToGo, startSpeed, cruiseSpeed, endSpeed);
                    TrajectoryFollower follower = new TrajectoryFollower(trajectoryToGo,CONST_TEST_PREFERRED_NEW_ANGLE);
                    this.m_Core.getChassis().addTask(follower);
                }
            }

            this.m_Core.updateTelemetry();
            telemetry.update();
        }
    }
}
