package org.firstinspires.ftc.teamcode.david_cao.Gen2_Lindel_Code.ComboKeys;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.david_cao.Gen2_Lindel_Code.LindelCore;

public class AUTO_StopSuckStonesCombo extends LindelComboKeyBase {
    public AUTO_StopSuckStonesCombo(LindelCore robotCore) {
        super(robotCore);
    }

    @Override
    protected void ___startCombo() {
        this.getRobotCore().setIntakeSystemStatus(LindelCore.IntakeSystemStatus.VOMIT,0.8);
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
        if (ms >= 400 && ms < 800) {
            this.getRobotCore().setIntakeSystemStatus(LindelCore.IntakeSystemStatus.SUCK, 0.8);
        } else if (ms >= 800 && ms < 1300) {
            this.getRobotCore().setIntakeSystemStatus(LindelCore.IntakeSystemStatus.STOP, 0);
            this.getRobotCore().setOrientServoToOrient(true);
        } else if (ms >= 1300) {
            this.getRobotCore().setOrientServoToOrient(false);
            this.getRobotCore().setGrabberServoToGrab(true);
            this.stopCombo();
        }
    }
}
