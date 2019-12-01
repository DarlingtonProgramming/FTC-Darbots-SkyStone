package org.darbots.darbotsftclib.testcases.MecanumPosTrackerTest;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.darbots.darbotsftclib.libcore.OpModes.DarbotsBasicOpMode;

@TeleOp(group = "DarbotsLib-TestCases", name = "MecanumPosTrackerTest")
public class MecnumPosTracker_TeleOp extends DarbotsBasicOpMode<MecanumPosTrackerTest_Core> {
    private MecanumPosTrackerTest_Core m_Core;
    @Override
    public MecanumPosTrackerTest_Core getRobotCore() {
        return null;
    }

    @Override
    public void hardwareInitialize() {
        this.m_Core = new MecanumPosTrackerTest_Core(this.hardwareMap);
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
            this.m_Core.updateTelemetry();
            telemetry.update();
        }
    }
}
