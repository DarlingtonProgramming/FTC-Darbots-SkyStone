package org.firstinspires.ftc.teamcode.david_cao.Gen2_Lindel_Code.ComboKeys;

import org.darbots.darbotsftclib.libcore.tasks.servo_tasks.motor_powered_servo_tasks.TargetPosTask;
import org.firstinspires.ftc.teamcode.david_cao.Gen2_Lindel_Code.LindelCore;
import org.firstinspires.ftc.teamcode.david_cao.Gen2_Lindel_Code.LindelSettings;

public class UNIVERSAL_BringElevatorDownCombo extends LindelComboKeyBase {
    public double targetSlideSpeed = 0.2;
    public double targetGrabberRotSpeed = 1.0;

    private int stage = 0;

    public UNIVERSAL_BringElevatorDownCombo(LindelCore robotCore) {
        super(robotCore);
    }

    @Override
    protected void ___startCombo() {
        stage = 0;
        double currentSlidePosition = this.getRobotCore().getLinearSlide().getCurrentPosition();
        if(currentSlidePosition >= LindelSettings.LINEAR_SLIDE_SAFE){
            this.getRobotCore().setGrabberRotServoToOutside(false,this.targetGrabberRotSpeed);
            stage = 1;
        }
        this.getRobotCore().getLinearSlide().replaceTask(new TargetPosTask(null,LindelSettings.LINEAR_SLIDE_SAFE,targetSlideSpeed));
    }

    @Override
    protected void ___stopCombo() {
        this.getRobotCore().getLinearSlide().deleteAllTasks();
        this.getRobotCore().getGrabberRotServo().stop();
    }

    @Override
    public void updateStatus() {
        if(!this.isBusy()){
            return;
        }
        if(stage == 0){
            if(!this.getRobotCore().getLinearSlide().isBusy()) {
                stage = 1;
                this.getRobotCore().setGrabberRotServoToOutside(false,this.targetGrabberRotSpeed);
            }
        }else if(stage == 1){
            if(!this.getRobotCore().getGrabberRotServo().isBusy()){
                stage = 2;
                this.getRobotCore().getLinearSlide().replaceTask(new TargetPosTask(null,0,this.targetSlideSpeed));
            }
        }else if(stage == 2){
            if(!this.getRobotCore().getLinearSlide().isBusy()){
                stage = 3;
            }
        }else{
            this.stopCombo();
        }
    }
}
