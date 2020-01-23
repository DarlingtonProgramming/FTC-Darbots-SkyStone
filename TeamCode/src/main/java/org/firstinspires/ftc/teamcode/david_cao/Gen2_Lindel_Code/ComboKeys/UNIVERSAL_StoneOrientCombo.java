package org.firstinspires.ftc.teamcode.david_cao.Gen2_Lindel_Code.ComboKeys;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.david_cao.Gen2_Lindel_Code.LindelCore;

public class UNIVERSAL_StoneOrientCombo extends LindelComboKeyBase {
    public UNIVERSAL_StoneOrientCombo(LindelCore robotCore) {
        super(robotCore);
    }

    @Override
    protected void ___startCombo() {
        this.getRobotCore().setOrientServoToOrient(true);
        this.getRobotCore().setGrabberServoToGrab(false);
    }

    @Override
    protected void ___stopCombo() {
        this.getRobotCore().setOrientServoToOrient(false);
        this.getRobotCore().setGrabberServoToGrab(true);
    }

    @Override
    public void updateStatus() {
        if(this.isBusy()) {
            if (this.getMilliSecondsSinceComboStart() >= 500) {
                this.stopCombo();
            }
        }
    }
}
