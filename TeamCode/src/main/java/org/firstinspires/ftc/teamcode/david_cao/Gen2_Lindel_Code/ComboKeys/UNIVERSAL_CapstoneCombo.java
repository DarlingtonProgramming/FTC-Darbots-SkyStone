package org.firstinspires.ftc.teamcode.david_cao.Gen2_Lindel_Code.ComboKeys;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.david_cao.Gen2_Lindel_Code.LindelCore;

public class UNIVERSAL_CapstoneCombo extends LindelComboKeyBase {
    public UNIVERSAL_CapstoneCombo(LindelCore robotCore) {
        super(robotCore);
    }

    @Override
    protected void ___startCombo() {
        this.getRobotCore().setCapStoneServoToDeposit(true);
    }

    @Override
    protected void ___stopCombo() {
        this.getRobotCore().setCapStoneServoToDeposit(false);
    }

    @Override
    public void updateStatus() {
        if(this.isBusy()){
            if(this.getMilliSecondsSinceComboStart() >= 800){
                this.stopCombo();
            }
        }
    }
}
