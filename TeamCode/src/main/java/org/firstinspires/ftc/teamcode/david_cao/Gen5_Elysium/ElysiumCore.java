package org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.darbots.darbotsftclib.libcore.templates.RobotCore;
import org.darbots.darbotsftclib.libcore.templates.chassis_related.RobotMotionSystem;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ElysiumCore extends RobotCore {
    public ElysiumCore(String logFileName, HardwareMap hardwareMap) {
        super(logFileName, hardwareMap);
    }

    public ElysiumCore(String logFileName, HardwareMap hardwareMap, int ThreadPriority) {
        super(logFileName, hardwareMap, ThreadPriority);
    }

    @Override
    protected void __stop() {

    }

    @Override
    protected void __terminate() {

    }

    @Override
    public RobotMotionSystem getChassis() {
        return null;
    }

    @Override
    protected void __updateStatus() {

    }

    @Override
    public void __updateTelemetry(Telemetry telemetry, TelemetryPacket telemetryPacket) {

    }

    @Override
    public boolean isBusy() {
        return false;
    }
}
