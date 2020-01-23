package org.firstinspires.ftc.teamcode.david_cao.Gen2_Lindel_Code.ComboKeys;

import com.qualcomm.robotcore.util.Range;

import org.darbots.darbotsftclib.libcore.runtime.MovementUtil;
import org.darbots.darbotsftclib.libcore.tasks.servo_tasks.motor_powered_servo_tasks.TargetPosTask;
import org.darbots.darbotsftclib.libcore.templates.chassis_related.MotionSystemConstraints;
import org.darbots.darbotsftclib.libcore.templates.chassis_related.RobotMotionSystemTask;
import org.darbots.darbotsftclib.season_specific.skystone.SkyStoneCoordinates;
import org.firstinspires.ftc.teamcode.david_cao.Gen2_Lindel_Code.LindelCore;
import org.firstinspires.ftc.teamcode.david_cao.Gen2_Lindel_Code.LindelSettings;

public class UNIVERSAL_AutoExitCombo extends LindelComboKeyBase {
    private int stage = 0;
    public double targetSlideSpeed = 0.2;
    public double targetChassisSpeed_Normalized = 0.5;
    public double targetChassisMaximumAccel_Normalized = 0.1;

    private double targetSpeed;
    private MotionSystemConstraints targetConstraint;
    private RobotMotionSystemTask originalTask = null;

    public UNIVERSAL_AutoExitCombo(LindelCore robotCore) {
        super(robotCore);
    }

    @Override
    protected void ___startCombo() {
        stage = 0;
        originalTask = this.getRobotCore().getChassis().getCurrentTask();
        this.getRobotCore().getChassis().deleteAllTasks();
        this.targetSpeed = this.targetChassisSpeed_Normalized * this.getRobotCore().getChassis().calculateMaxLinearXSpeedInCMPerSec();
        this.targetConstraint = this.getRobotCore().getChassis().getMotionSystemConstraints(this.targetChassisMaximumAccel_Normalized * this.getRobotCore().getChassis().calculateMaxLinearXSpeedInCMPerSec(),0,0,0);
        double currentLinearSlidePos = this.getRobotCore().getLinearSlide().getCurrentPosition();
        double targetLinearSlidePos = currentLinearSlidePos - LindelSettings.CONTROL_STONE_PROMINENCE_HEIGHT_SLIDE;
        targetLinearSlidePos = Range.clip(targetLinearSlidePos,LindelSettings.LINEAR_SLIDE_MIN,LindelSettings.LINEAR_SLIDE_MAX);
        this.getRobotCore().getLinearSlide().replaceTask(new TargetPosTask(null, targetLinearSlidePos,this.targetSlideSpeed));
    }

    @Override
    protected void ___stopCombo() {
        this.getRobotCore().getLinearSlide().deleteAllTasks();
        if(this.getRobotCore().getChassis().getCurrentTask() != originalTask){
            this.getRobotCore().getChassis().replaceTask(originalTask);
        }
    }

    @Override
    public void updateStatus() {
        if(stage == 0){
            if(!this.getRobotCore().getLinearSlide().isBusy()){
                stage = 1;
                this.getRobotCore().setGrabberServoToGrab(false);
                this.getRobotCore().getChassis().replaceTask(MovementUtil.getGoToPointTask(SkyStoneCoordinates.STONE_LENGTH,0,0,this.targetSpeed,0,0));
            }
        }else if(stage == 1){
            if(!this.getRobotCore().getChassis().isBusy()){
                stage = 2;
            }
        }else{
            this.stopCombo();
        }
    }
}
