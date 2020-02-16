package org.firstinspires.ftc.teamcode.david_cao.Gen5_Elysium;

import org.darbots.darbotsftclib.libcore.OpModes.DarbotsBasicOpMode;
import org.darbots.darbotsftclib.libcore.runtime.GlobalMedia;
import org.darbots.darbotsftclib.libcore.templates.RobotNonBlockingDevice;

public abstract class ElysiumSoundBox implements RobotNonBlockingDevice {
    public DarbotsBasicOpMode<ElysiumCore> opMode;
    public ElysiumSoundBox(DarbotsBasicOpMode<ElysiumCore> opMode){
        this.opMode = opMode;
    }
    public abstract void __onInitialize();
    public abstract void __onStart();
    public abstract void __onLoop(double secondsSinceOpModeStart);
    public abstract void __terminate();
    public void terminate(){
        this.__terminate();
        GlobalMedia.stopPlayingAllSounds();
        GlobalMedia.stopPlayingLoopSounds();
    }
    public void onInitialize(){
        this.__onInitialize();
    }
    public void onStart(){
        this.__onStart();
    }
    public void updateStatus(){
        this.__onLoop(this.opMode.getSecondsSinceOpModeStarted());
    }
    public boolean isBusy(){
        return false;
    }
    public void waitUntilFinish(){
        return;
    }
}
