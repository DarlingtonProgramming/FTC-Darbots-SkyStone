package org.darbots.darbotsftclib.testcases.MecanumPosTrackerTest;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.darbots.darbotsftclib.libcore_4_5_0Pre.OpModes.DarbotsBasicOpMode;
import org.darbots.darbotsftclib.testcases.common.TestMecanumCore;

@TeleOp(group = "DarbotsLib-TestCases", name = "MecanumPosTrackerTest")
@Disabled
public class MecnumPosTracker_TeleOp extends DarbotsBasicOpMode<TestMecanumCore> {
    private TestMecanumCore m_Core;
    @Override
    public TestMecanumCore getRobotCore() {
        return m_Core;
    }

    @Override
    public void hardwareInitialize() {
        this.m_Core = new TestMecanumCore(this.hardwareMap,"MecanumPosTrackerTest.log");
    }

    @Override
    public void hardwareDestroy() {

    }

    @Override
    public void RunThisOpMode() {
        while(this.opModeIsActive()){
            this.m_Core.updateStatus();
            this.m_Core.updateTelemetry();
            telemetry.update();
        }
    }
}
