package org.firstinspires.ftc.teamcode.david_cao.Gen2_Lindel_Code.ComboKeys;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.darbots.darbotsftclib.libcore.tasks.servo_tasks.motor_powered_servo_tasks.TargetPosTask;
import org.firstinspires.ftc.teamcode.david_cao.Gen2_Lindel_Code.LindelCore;
import org.firstinspires.ftc.teamcode.david_cao.Gen2_Lindel_Code.LindelSettings;

public class AUTO_DropStoneToFoundationCombo extends LindelComboKeyBase {
    private int stage;

    public AUTO_DropStoneToFoundationCombo(LindelCore robotCore) {
        super(robotCore);
    }

    @Override
    protected void ___startCombo() {
        stage = 0;
        this.getRobotCore().setGrabberServoToGrab(true);
        this.getRobotCore().getLinearSlide().replaceTask(new TargetPosTask(null, LindelSettings.LINEAR_SLIDE_SAFE,1.0));
    }

    @Override
    protected void ___stopCombo() {

    }

    @Override
    public void updateStatus() {
        if(!this.isBusy()){
            return;
        }
        double ms = this.getMilliSecondsSinceComboStart();
        if(stage == 0){
            if(!this.getRobotCore().getLinearSlide().isBusy()){
                stage = 1;
                this.getRobotCore().setGrabberRotServoToOutside(true,1);
            }
        }else if(stage == 1){
            if(!this.getRobotCore().getGrabberRotServo().isBusy()){
                stage = 2;
                this.getRobotCore().getLinearSlide().replaceTask(new TargetPosTask(null,this.getRobotCore().getLinearSlide().getMinPos(),1.0));
            }
        }else if(stage == 2){
            if(!this.getRobotCore().getLinearSlide().isBusy()){
                stage = 3;
                this.getRobotCore().setGrabberServoToGrab(false);
                this.getRobotCore().getLinearSlide().replaceTask(new TargetPosTask(null,LindelSettings.LINEAR_SLIDE_SAFE,1.0));
            }
        }else if(stage == 3){
            if(!this.getRobotCore().getLinearSlide().isBusy()){
                stage = 4;
                this.getRobotCore().setGrabberRotServoToOutside(false,1.0);
            }
        }else if(stage == 4){
            if(!this.getRobotCore().getGrabberRotServo().isBusy()){
                stage = 5;
                this.getRobotCore().setGrabberServoToGrab(true);
                this.getRobotCore().getLinearSlide().replaceTask(new TargetPosTask(null,LindelSettings.LINEAR_SLIDE_INIT,1.0));
            }
        }else{
            this.stopCombo();
        }
    }
}
