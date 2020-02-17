package org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.Soundboxes;

import org.darbots.darbotsftclib.libcore.OpModes.DarbotsBasicOpMode;
import org.darbots.darbotsftclib.libcore.runtime.GlobalMedia;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.ElysiumCore;

public class ElysiumAutoSoundBox extends ElysiumGlobalSoundBox {
    public ElysiumAutoSoundBox(DarbotsBasicOpMode opMode,  ElysiumCore core) {
        super(opMode, core);
    }

    @Override
    public void ___onInitialize() {
        super.secondsForOpModeRun = 30.0;
        GlobalMedia.preloadResourceFile(super.opMode.hardwareMap.appContext, R.raw.auto_bgm);
        GlobalMedia.preloadResourceFile(super.opMode.hardwareMap.appContext,R.raw.auto_drag);
        GlobalMedia.preloadResourceFile(super.opMode.hardwareMap.appContext,R.raw.auto_get_stone);
        GlobalMedia.preloadResourceFile(super.opMode.hardwareMap.appContext,R.raw.auto_stone_down);
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
        GlobalMedia.playResourceFile(super.opMode.hardwareMap.appContext,false,R.raw.auto_bgm,1.0f,1.0f,false,0);
    }

    public void onGrabbingFoundation(){
        GlobalMedia.playResourceFile(super.opMode.hardwareMap.appContext,false,R.raw.auto_drag,1.0f,1.0f,false,0);
    }

    public void onGrabbingStone(){
        GlobalMedia.playResourceFile(super.opMode.hardwareMap.appContext,false,R.raw.auto_get_stone,1.0f,1.0f,false,0);
    }

    public void onReleasingStone(){
        GlobalMedia.playResourceFile(super.opMode.hardwareMap.appContext,false,R.raw.auto_stone_down,1.0f,1.0f,false,0);
    }

    @Override
    public void __terminate() {

    }
}
