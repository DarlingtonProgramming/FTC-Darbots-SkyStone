package org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.Soundboxes;

import org.darbots.darbotsftclib.libcore.OpModes.DarbotsBasicOpMode;
import org.darbots.darbotsftclib.libcore.runtime.GlobalMedia;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.ElysiumCore;
import org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium.ElysiumSoundBox;

public abstract class ElysiumGlobalSoundBox extends ElysiumSoundBox {
    public ElysiumGlobalSoundBox(DarbotsBasicOpMode<ElysiumCore> opMode) {
        super(opMode);
    }
    protected double secondsForOpModeRun = 30.0;
    private boolean endingSoundPlayed = false;
    private static final double ENDING_SOUND_SECONDS = 5.5;

    @Override
    public void __onInitialize() {
        GlobalMedia.playResourceFile(super.opMode.hardwareMap.appContext,false, R.raw.global_initialization,1.0f,1.0f,false,0);
        GlobalMedia.preloadResourceFile(super.opMode.hardwareMap.appContext,R.raw.global_shutdown);
        this.___onInitialize();
    }

    public abstract void ___onInitialize();

    @Override
    public void __onLoop(double secondsSinceOpModeStart) {
        if(secondsSinceOpModeStart >= secondsForOpModeRun - ENDING_SOUND_SECONDS){
            if(!endingSoundPlayed){
                this.__beforeEnding();
                GlobalMedia.playResourceFile(super.opMode.hardwareMap.appContext,false, R.raw.global_shutdown,1.0f,1.0f,false,0);
                endingSoundPlayed = true;
            }
        }
    }

    public abstract void ___onLoop(double secondsSinceOpModeStart);
    public abstract void __beforeEnding();
}
