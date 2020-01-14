package org.firstinspires.ftc.teamcode.david_cao.Gen4_SwanSilver_Code;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.darbots.darbotsftclib.libcore.OpModes.DarbotsBasicOpMode;

@TeleOp(group = "4100", name = "4100TeleOpEmpty")
public class SwanSilverEmptyTeleOp extends DarbotsBasicOpMode<SwanSilverCore> {
    private SwanSilverCore m_Core;
    @Override
    public SwanSilverCore getRobotCore() {
        return this.m_Core;
    }

    @Override
    public void hardwareInitialize() {
        this.m_Core = new SwanSilverCore(this.hardwareMap,"SwanSilverEmptyTeleOp.log");
    }

    @Override
    public void hardwareDestroy() {

    }

    @Override
    public void RunThisOpMode() {
        while(this.opModeIsActive()){
            this.m_Core.updateStatus();
            this.m_Core.updateTelemetry();
            this.telemetry.update();
        }
    }
}
