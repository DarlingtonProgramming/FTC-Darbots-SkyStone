package org.firstinspires.ftc.teamcode.david_cao.Gen2_Lindel_Code.ComboKeys;

import org.darbots.darbotsftclib.libcore.tasks.servo_tasks.motor_powered_servo_tasks.TargetPosSpeedCtlTask;
import org.darbots.darbotsftclib.libcore.tasks.servo_tasks.motor_powered_servo_tasks.TargetPosTask;
import org.firstinspires.ftc.teamcode.david_cao.Gen2_Lindel_Code.LindelCore;
import org.firstinspires.ftc.teamcode.david_cao.Gen2_Lindel_Code.LindelSettings;
import org.firstinspires.ftc.teamcode.robot_common.Robot4100Common;

public class UNIVERSAL_AutoElevatorCombo extends LindelComboKeyBase {
    public double targetPosition = 0.0;
    public double targetSlideSpeed = 1.0;
    public double targetGrabberRotSpeed = 1.0;
    private int stage = 0;
    public UNIVERSAL_AutoElevatorCombo(LindelCore robotCore) {
        super(robotCore);
    }

    @Override
    protected void ___startCombo() {
        stage = 0;
        this.getRobotCore().setGrabberServoToGrab(true);
        double slideCurrentPosition = this.getRobotCore().getLinearSlide().getCurrentPosition();
        if(slideCurrentPosition < LindelSettings.LINEAR_SLIDE_SAFE){
            this.getRobotCore().getLinearSlide().replaceTask(new TargetPosSpeedCtlTask(null,LindelSettings.LINEAR_SLIDE_SAFE,this.targetSlideSpeed));
        }
    }

    @Override
    protected void ___stopCombo() {
        this.getRobotCore().getLinearSlide().deleteAllTasks();
        this.getRobotCore().getGrabberRotServo().stop();
    }

    @Override
    public void updateStatus() {
        if(stage == 0){
            if(!this.getRobotCore().getLinearSlide().isBusy()){
                stage = 1;
                this.getRobotCore().setGrabberRotServoToOutside(true,this.targetGrabberRotSpeed);
                if(this.targetPosition >= LindelSettings.LINEAR_SLIDE_SAFE){
                    this.getRobotCore().getLinearSlide().replaceTask(new TargetPosTask(null,this.targetPosition,this.targetSlideSpeed));
                }
            }
        }else if(stage == 1){
            if(!this.getRobotCore().getGrabberRotServo().isBusy()){
                if(this.targetPosition < LindelSettings.LINEAR_SLIDE_SAFE){
                    this.getRobotCore().getLinearSlide().replaceTask(new TargetPosTask(null, this.targetPosition, this.targetSlideSpeed));
                    stage = 2;
                }else{
                    stage = 3;
                }
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
