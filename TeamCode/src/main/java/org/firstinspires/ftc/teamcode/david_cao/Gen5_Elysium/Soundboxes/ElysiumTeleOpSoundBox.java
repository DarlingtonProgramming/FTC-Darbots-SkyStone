package org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.Soundboxes;

import org.darbots.darbotsftclib.libcore.OpModes.DarbotsBasicOpMode;
import org.darbots.darbotsftclib.libcore.runtime.GlobalMedia;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.ElysiumCore;

public class ElysiumTeleOpSoundBox extends ElysiumGlobalSoundBox {
    public ElysiumTeleOpSoundBox(DarbotsBasicOpMode<ElysiumCore> opMode) {
        super(opMode);
    }

    @Override
    public void ___onInitialize() {
        super.secondsForOpModeRun = 2 * 60.0;
        GlobalMedia.preloadResourceFile(super.opMode.hardwareMap.appContext, R.raw.teleop_bgm);
    }

    @Override
    public void ___onLoop(double secondsSinceOpModeStart) {

    }

    @Override
    public void __beforeEnding() {
        GlobalMedia.stopPlayingLoopSounds();
    }

    @Override
    public void __onStart() {
        GlobalMedia.playResourceFile(super.opMode.hardwareMap.appContext,false,R.raw.teleop_bgm,1.0f,1.0f,true,-1);
    }

    @Override
    public void __terminate() {

    }
}
