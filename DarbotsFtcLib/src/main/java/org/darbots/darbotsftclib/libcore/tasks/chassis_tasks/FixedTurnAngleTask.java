package org.darbots.darbotsftclib.libcore.tasks.chassis_tasks;

import com.acmerobotics.dashboard.canvas.Canvas;

import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPose2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotVector2D;
import org.darbots.darbotsftclib.libcore.motion_planning.profiles.MotionProfile;
import org.darbots.darbotsftclib.libcore.motion_planning.profiles.MotionProfileGenerator;
import org.darbots.darbotsftclib.libcore.motion_planning.profiles.MotionProfileIterator;
import org.darbots.darbotsftclib.libcore.motion_planning.profiles.MotionState;
import org.darbots.darbotsftclib.libcore.templates.chassis_related.MotionSystemConstraints;
import org.darbots.darbotsftclib.libcore.templates.chassis_related.RobotMotionSystemTask;

public class FixedTurnAngleTask extends RobotMotionSystemTask {
    public double angleToTurn;
    public double startAngularVelocity;
    public double endAngularVelocity;
    public double cruiseAngularVelocity;
    public MotionSystemConstraints motionSystemConstraints;
    private MotionProfile m_MotionProfile = null;
    private MotionProfileIterator m_ProfileIterator = null;

    public FixedTurnAngleTask(double angleToTurn, double startAngularVelocity, double cruiseAngularVelocity, double endAngularVelocity, MotionSystemConstraints motionSystemConstraints){
        this.angleToTurn = angleToTurn;
        this.startAngularVelocity = startAngularVelocity;
        this.endAngularVelocity = endAngularVelocity;
        this.cruiseAngularVelocity = cruiseAngularVelocity;
        this.motionSystemConstraints = motionSystemConstraints;
    }

    public FixedTurnAngleTask(FixedTurnAngleTask oldTask){
        this.angleToTurn = oldTask.angleToTurn;
        this.startAngularVelocity = oldTask.startAngularVelocity;
        this.cruiseAngularVelocity = oldTask.cruiseAngularVelocity;
        this.endAngularVelocity = oldTask.endAngularVelocity;
        this.motionSystemConstraints = oldTask.motionSystemConstraints;
    }

    @Override
    protected void __startTask() {
        this.m_MotionProfile = MotionProfileGenerator.generateAngularMotionProfile(motionSystemConstraints,angleToTurn,startAngularVelocity,cruiseAngularVelocity,endAngularVelocity);
        this.m_ProfileIterator = new MotionProfileIterator(this.m_MotionProfile);
    }

    @Override
    protected void __taskFinished() {

    }

    @Override
    protected void __updateStatus() {
        double currentTime = this.getSecondsSinceTaskStart();
        if(currentTime > this.m_ProfileIterator.getTotalDuration()){
            this.stopTask();
            return;
        }
        double timeToForward = currentTime - this.m_ProfileIterator.getCurrentDuration();
        MotionState currentState = this.m_ProfileIterator.forward(timeToForward);
        this.setRobotSpeed(new RobotVector2D(0,0,currentState.velocity),new RobotPose2D(0,0,currentState.distance));
    }

    @Override
    protected RobotPose2D __getSupposedTaskFinishPos() {
        return new RobotPose2D(0,0,angleToTurn);
    }

    @Override
    public void drawPath(Canvas dashboardCanvas) {

    }
}
