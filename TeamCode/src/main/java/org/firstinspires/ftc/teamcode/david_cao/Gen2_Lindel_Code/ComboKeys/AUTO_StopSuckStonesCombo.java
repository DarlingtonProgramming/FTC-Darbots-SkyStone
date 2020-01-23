package org.firstinspires.ftc.teamcode.david_cao.Gen2_Lindel_Code.ComboKeys;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.david_cao.Gen2_Lindel_Code.LindelCore;

public class AUTO_StopSuckStonesCombo extends LindelComboKeyBase {
    public AUTO_StopSuckStonesCombo(LindelCore robotCore) {
        super(robotCore);
    }

    @Override
    protected void ___startCombo() {
        this.getRobotCore().setIntakeSystemStatus(LindelCore.IntakeSystemStatus.VOMIT,0.3);
        this.getRobotCore().setGrabberServoToGrab(false);
    }

    @Override
    protected void ___stopCombo() {
        this.getRobotCore().setIntakeSystemStatus(LindelCore.IntakeSystemStatus.STOP,0);
        this.getRobotCore().setGrabberServoToGrab(true);
        this.getRobotCore().setOrientServoToOrient(false);
    }

    @Override
    public void updateStatus() {
        if (!this.isBusy()) {
            return;
        }
        double ms = this.getMilliSecondsSinceComboStart();
        if (ms >= 200 && ms < 500) {
            this.getRobotCore().setIntakeSystemStatus(LindelCore.IntakeSystemStatus.SUCK, 0.4);
        } else if (ms >= 500 && ms < 1000) {
            this.getRobotCore().setIntakeSystemStatus(LindelCore.IntakeSystemStatus.STOP, 0);
            this.getRobotCore().setOrientServoToOrient(true);
        } else if (ms >= 1000) {
            this.getRobotCore().setOrientServoToOrient(false);
            this.getRobotCore().setGrabberServoToGrab(true);
            this.stopCombo();
        }
    }
}
