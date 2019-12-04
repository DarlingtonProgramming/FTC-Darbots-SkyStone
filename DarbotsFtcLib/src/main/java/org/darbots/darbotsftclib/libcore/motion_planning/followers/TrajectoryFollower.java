package org.darbots.darbotsftclib.libcore.motion_planning.followers;

import com.qualcomm.robotcore.util.Range;

import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPoint2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPose2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotVector2D;
import org.darbots.darbotsftclib.libcore.motion_planning.profiles.MotionState;
import org.darbots.darbotsftclib.libcore.motion_planning.trajectories.TrajectoryMotionState;
import org.darbots.darbotsftclib.libcore.templates.chassis_related.RobotMotionSystemTask;
import org.darbots.darbotsftclib.libcore.templates.motion_planning.RobotMotionProfilingIterator;
import org.darbots.darbotsftclib.libcore.templates.motion_planning.RobotTrajectory;

public class TrajectoryFollower extends RobotMotionSystemTask {
    private RobotTrajectory m_Trajectory;
    private RobotMotionProfilingIterator<TrajectoryMotionState,?> m_TrajectoryIterator;
    private RobotPoint2D m_TargetEndPoint;
    private RobotPose2D m_TargetEndPose;
    private double m_TotalDuration;

    public TrajectoryFollower(RobotTrajectory trajectory, double preferredEndAngle){
        super();
        this.m_Trajectory = trajectory;
        this.m_TargetEndPose = new RobotPose2D(0,0,preferredEndAngle);
        this.__updateTrajectory();
    }

    public TrajectoryFollower(TrajectoryFollower oldFollower){
        super(oldFollower);
        this.m_Trajectory = oldFollower.m_Trajectory;
        this.m_TargetEndPose = new RobotPose2D(0,0,oldFollower.m_TargetEndPose.getRotationZ());
        this.__updateTrajectory();
    }

    protected void __updateTrajectory(){
        this.m_TargetEndPoint = this.m_Trajectory.getPath().getPointAt(this.m_Trajectory.getPath().getIndependentVariableExtreme());
        this.m_TargetEndPose.X = this.m_TargetEndPoint.X;
        this.m_TargetEndPose.Y = this.m_TargetEndPoint.Y;
    }

    public double getPerferredEndAngle(){
        return this.m_TargetEndPose.getRotationZ();
    }

    public void setPreferredEndAngle(double endAngle){
        this.m_TargetEndPose.setRotationZ(endAngle);
    }

    @Override
    protected void __startTask() {
        this.m_TrajectoryIterator = m_Trajectory.getIterator();
        this.m_TotalDuration = this.m_TrajectoryIterator.getTotalDuration();
    }

    @Override
    protected void __taskFinished() {
        this.m_TrajectoryIterator = null;
    }

    @Override
    protected void __updateStatus() {
        double currentTime = this.getSecondsSinceTaskStart();
        if(currentTime > this.m_TotalDuration){
            this.stopTask();
            return;
        }
        TrajectoryMotionState supposedMotionState = this.m_TrajectoryIterator.forward(currentTime - this.m_TrajectoryIterator.getCurrentDuration());
        RobotPose2D supposedDistancePose = new RobotPose2D(supposedMotionState.xDisplacement,supposedMotionState.yDisplacement,this.m_TargetEndPose.getRotationZ());
        RobotVector2D correctionPose = this.getErrorCorrectionVelocityVector(supposedDistancePose);
        RobotVector2D afterCorrectionVelocity = new RobotPose2D(supposedMotionState.xVelocity+correctionPose.X,supposedMotionState.yVelocity+correctionPose.Y,0);
        RobotVector2D motionSystemMaxVelocity = this.getMotionSystem().getTheoreticalMaximumMotionState(afterCorrectionVelocity);
        RobotVector2D actualVelocity = null;
        if(Math.abs(afterCorrectionVelocity.X) > Math.abs(motionSystemMaxVelocity.X) || Math.abs(afterCorrectionVelocity.Y) > Math.abs(motionSystemMaxVelocity.Y)){
            actualVelocity = new RobotVector2D(motionSystemMaxVelocity);
        }else{
            actualVelocity = new RobotVector2D(afterCorrectionVelocity);
        }
        double possibleMaxZRot = this.getMotionSystem().calculateMaxAngularSpeedInDegPerSec(Math.abs(actualVelocity.X) + Math.abs(actualVelocity.Y));
        double rotZCorrectionVelocity = Range.clip(correctionPose.getRotationZ(),-possibleMaxZRot,possibleMaxZRot);
        actualVelocity.setRotationZ(rotZCorrectionVelocity);
        this.getMotionSystem().setRobotSpeed(actualVelocity);
    }

    @Override
    protected RobotPose2D __getSupposedTaskFinishPos() {
        return this.m_TargetEndPose;
    }
}
