package org.firstinspires.ftc.teamcode.david_cao.Gen2_Lindel_Code.ComboKeys;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.darbots.darbotsftclib.libcore.templates.DarbotsComboKey;
import org.firstinspires.ftc.teamcode.david_cao.Gen2_Lindel_Code.LindelCore;

public abstract class LindelComboKeyBase extends DarbotsComboKey
{
    private ElapsedTime time;
    private LindelCore core;
    public LindelComboKeyBase(LindelCore robotCore){
        this.core = robotCore;
    }
    protected LindelCore getRobotCore(){
        return this.core;
    }
    protected double getMilliSecondsSinceComboStart(){
        return time.milliseconds();
    }
    @Override
    protected void __startCombo() {
        time = new ElapsedTime();
        ___startCombo();
    }

    protected abstract void ___startCombo();

    @Override
    protected void __stopCombo() {
        this.___stopCombo();
        time = null;
    }

    protected abstract void ___stopCombo();
}
