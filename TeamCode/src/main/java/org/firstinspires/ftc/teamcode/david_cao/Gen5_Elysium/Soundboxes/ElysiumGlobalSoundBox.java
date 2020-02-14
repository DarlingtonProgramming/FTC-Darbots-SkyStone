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
    private static final double ENDINGSOUND_SECONDS = 6.0;

    @Override
    public void __onInitialize() {
        GlobalMedia.playResourceFile(super.opMode.hardwareMap.appContext,false, R.raw.global_initialization,1.0f,1.0f,false,0);
    }

    @Override
    public void __onLoop(double secondsSinceOpModeStart) {
        if(secondsSinceOpModeStart >= secondsForOpModeRun - ENDINGSOUND_SECONDS){
            if(!endingSoundPlayed){
                GlobalMedia.playResourceFile(super.opMode.hardwareMap.appContext,false, R.raw.global_shutdown,1.0f,1.0f,false,0);
                endingSoundPlayed = true;
            }
        }
    }

    public abstract void ___onLoop(double secondsSinceOpModeStart);
}
