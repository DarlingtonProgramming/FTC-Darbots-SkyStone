package org.darbots.darbotsftclib.libcore.purepursuit.followers;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.robotcore.util.Range;

import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPoint2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPose2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.XYPlaneCalculations;
import org.darbots.darbotsftclib.libcore.purepursuit.waypoints.PurePursuitEndPoint;
import org.darbots.darbotsftclib.libcore.purepursuit.waypoints.PurePursuitHeadingInterpolationWayPoint;
import org.darbots.darbotsftclib.libcore.purepursuit.waypoints.PurePursuitWayPoint;
import org.darbots.darbotsftclib.libcore.templates.DarbotsAction;
import org.darbots.darbotsftclib.libcore.templates.chassis_related.RobotMotionSystemTask;

import java.util.ArrayList;

public class PurePursuitPathFollower extends RobotMotionSystemTask {
    public static class FollowInformation{
        public PurePursuitWayPoint purePursuitWayPoint;
        public RobotPoint2D pursuitPoint;
        public double normalizedFollowSpeed;

        public FollowInformation(PurePursuitWayPoint wayPoint, RobotPoint2D followPoint, double normalizedFollowSpeed){
            this.pursuitPoint = followPoint;
            this.purePursuitWayPoint = wayPoint;
            this.normalizedFollowSpeed = Math.abs(normalizedFollowSpeed);
        }
        public FollowInformation(FollowInformation oldInfo){
            this.pursuitPoint = oldInfo.pursuitPoint;
            this.purePursuitWayPoint = oldInfo.purePursuitWayPoint;
            this.normalizedFollowSpeed = oldInfo.normalizedFollowSpeed;
        }
    }

    protected ArrayList<PurePursuitWayPoint> m_Path;
    protected int m_PathCursor;
    protected double m_FollowSpeedNormalized;
    protected double m_AngleSpeedNormalized;
    protected double m_PreferredAngle;
    protected double m_FollowStartSpeedNormalized;
    protected double m_SpeedThreshold = 0.05;

    public PurePursuitPathFollower(ArrayList<PurePursuitWayPoint> path, double normalizedStartSpeed, double normalizedFollowSpeed, double normalizedAngleSpeed, double preferredAngle){
        this.m_Path = new ArrayList<>();
        this.m_Path.addAll(path);
        this.setFollowSpeed(normalizedFollowSpeed);
        this.setStartSpeed(normalizedStartSpeed);
        this.setAngleSpeed(normalizedAngleSpeed);
        this.setPreferredAngle(preferredAngle);
    }

    public PurePursuitPathFollower(PurePursuitPathFollower oldFollower){
        super(oldFollower);
        this.m_Path = new ArrayList<>();
        this.m_Path.addAll(oldFollower.m_Path);
        this.m_FollowSpeedNormalized = oldFollower.m_FollowSpeedNormalized;
        this.m_AngleSpeedNormalized = oldFollower.m_AngleSpeedNormalized;
        this.m_PreferredAngle = oldFollower.m_PreferredAngle;
        this.m_FollowStartSpeedNormalized = oldFollower.m_FollowStartSpeedNormalized;
        this.m_SpeedThreshold = oldFollower.m_SpeedThreshold;
    }

    public double getSpeedThreshold(){
        return this.m_SpeedThreshold;
    }

    public void setSpeedThreshold(double speedThreshold){
        this.m_SpeedThreshold = Math.abs(speedThreshold);
    }

    public double getStartSpeed(){
        return this.m_FollowStartSpeedNormalized;
    }

    public void setStartSpeed(double normalizedStartSpeed){
        this.m_FollowStartSpeedNormalized = Math.abs(normalizedStartSpeed);
    }

    public double getFollowSpeed(){
        return this.m_FollowSpeedNormalized;
    }

    public void setFollowSpeed(double normalizedFollowSpeed){
        this.m_FollowSpeedNormalized = Range.clip(Math.abs(normalizedFollowSpeed),0.0,1.0);
    }

    public double getAngleSpeed(){
        return this.m_AngleSpeedNormalized;
    }

    public void setAngleSpeed(double normalizedAngleSpeed){
        this.m_AngleSpeedNormalized = Range.clip(Math.abs(normalizedAngleSpeed),0.0,1.0);
    }

    public double getPreferredAngle(){
        return this.m_PreferredAngle;
    }

    public void setPreferredAngle(double preferredAngle){
        this.m_PreferredAngle = XYPlaneCalculations.normalizeDeg(preferredAngle);
    }

    public ArrayList<PurePursuitWayPoint> getPath(){
        return this.m_Path;
    }

    @Override
    protected void __startTask() {
        this.m_PathCursor = -1;
    }

    @Override
    protected void __taskFinished() {

    }

    @Override
    protected void __updateStatus() {
        RobotPose2D currentOffset = this.getRelativePositionOffsetSinceStart();
        FollowInformation followInfo = this.getFollowInformation(currentOffset);
        if(followInfo == null){
            this.stopTask();
            return;
        }
        if(followInfo.purePursuitWayPoint.SegmentBeginAction != null){
            if(!followInfo.purePursuitWayPoint.SegmentBeginAction.isActionFinished() && !followInfo.purePursuitWayPoint.SegmentBeginAction.isBusy()){
                followInfo.purePursuitWayPoint.SegmentBeginAction.startAction();
            }
            if(followInfo.purePursuitWayPoint.SegmentBeginAction.isBusy()){
                followInfo.purePursuitWayPoint.SegmentBeginAction.updateStatus();
            }
        }
        double followSpeed = followInfo.normalizedFollowSpeed * this.m_FollowSpeedNormalized;
        if(followSpeed < this.m_SpeedThreshold){
            followSpeed = this.m_SpeedThreshold;
        }
        this.gotoPoint(currentOffset,followInfo.purePursuitWayPoint,followInfo.pursuitPoint,followSpeed,this.m_AngleSpeedNormalized,this.m_PreferredAngle);
    }

    public void gotoPoint(RobotPose2D currentOffset, PurePursuitWayPoint pursuitWayPoint, RobotPoint2D pursuitPoint, double pursuitSpeedNormalized, double angleSpeedNormalized, double preferredAngle){
        RobotPoint2D currentRobotAxisTarget = XYPlaneCalculations.getRelativePosition(currentOffset,pursuitPoint);
        double targetPointAngle = Math.toDegrees(Math.atan2(currentRobotAxisTarget.Y,currentRobotAxisTarget.X));
        double wantedAngleInCurrentRobotAxis = targetPointAngle + preferredAngle;
        double wantedAngleInOriginalRobotAxis = preferredAngle;
        if(pursuitWayPoint instanceof PurePursuitHeadingInterpolationWayPoint){
            double preferredAngleInOriginalRobotAxis = ((PurePursuitHeadingInterpolationWayPoint) pursuitWayPoint).getDesiredHeading();
            wantedAngleInCurrentRobotAxis = preferredAngleInOriginalRobotAxis - currentOffset.getRotationZ();
            wantedAngleInOriginalRobotAxis = preferredAngleInOriginalRobotAxis;
        }else if(pursuitWayPoint instanceof PurePursuitEndPoint){
            PurePursuitEndPoint endPoint = (PurePursuitEndPoint) pursuitWayPoint;
            if(endPoint.headingInterpolationEnabled){
                double preferredAngleInOriginalRobotAxis = endPoint.getDesiredHeading();
                wantedAngleInCurrentRobotAxis = preferredAngleInOriginalRobotAxis - currentOffset.getRotationZ();
                wantedAngleInOriginalRobotAxis = preferredAngleInOriginalRobotAxis;
            }
        }

        double distToTargetWaypoint = currentOffset.toPoint2D().distanceTo(pursuitWayPoint);
        if(pursuitWayPoint instanceof PurePursuitEndPoint){
            if(distToTargetWaypoint < ((PurePursuitEndPoint) pursuitWayPoint).getEndErrorToleranceDistance()){
                pursuitSpeedNormalized = 0;
            }
        }else{
            if(distToTargetWaypoint < Math.min(pursuitWayPoint.getFollowDistance(),3)){
                pursuitSpeedNormalized = 0;
            }
        }

        RobotPose2D supposedPose = new RobotPose2D(pursuitPoint,wantedAngleInOriginalRobotAxis);
        this.updateSupposedPos(supposedPose);

        double xSpeed = currentRobotAxisTarget.X, ySpeed = currentRobotAxisTarget.Y;
        double tempSpeed = Math.abs(currentRobotAxisTarget.X) + Math.abs(currentRobotAxisTarget.Y);
        double tempFactor = pursuitSpeedNormalized / tempSpeed;
        xSpeed *= tempFactor;
        ySpeed *= tempFactor;
        double angularSpeed = Range.clip(wantedAngleInCurrentRobotAxis / 30.0,-1.0,1.0) * angleSpeedNormalized;

        this.getMotionSystem().setNormalizedRobotSpeed(
                xSpeed,
                ySpeed,
                angularSpeed
        );
    }

    public FollowInformation getFollowInformation(RobotPose2D currentOffset){

        // Check whether we should advance to the next piece of the curve
        boolean jumpToNextSegment = false;
        double distToTarget;
        do {
            jumpToNextSegment = false;
            PurePursuitWayPoint target = this.m_Path.get(this.m_PathCursor + 1);
            DarbotsAction targetAction = target.SegmentBeginAction;
            distToTarget = currentOffset.toPoint2D().distanceTo(target);
            if (target instanceof PurePursuitEndPoint) {
                PurePursuitEndPoint targetEnd = (PurePursuitEndPoint) target;
                if (distToTarget <= targetEnd.getEndErrorToleranceDistance()) {
                    jumpToNextSegment = true;
                }
                if(targetEnd.headingInterpolationEnabled) {
                    if (!(Math.abs(XYPlaneCalculations.normalizeDeg(currentOffset.getRotationZ() - targetEnd.getDesiredHeading())) <= targetEnd.getAllowedHeadingError())) {
                        jumpToNextSegment = false;
                    }
                }
            }else if (target instanceof PurePursuitHeadingInterpolationWayPoint) {
                PurePursuitHeadingInterpolationWayPoint ptTarget = (PurePursuitHeadingInterpolationWayPoint) target;
                if ((Math.abs(XYPlaneCalculations.normalizeDeg(currentOffset.getRotationZ() - ptTarget.getDesiredHeading())) <= ptTarget.getAllowedHeadingError())) {
                    jumpToNextSegment = true;
                }
                if(distToTarget > target.getFollowDistance()){
                    jumpToNextSegment = false;
                }
            }else {
                if (distToTarget <= target.getFollowDistance()) {
                    jumpToNextSegment = true;
                }
            }

            if(jumpToNextSegment && targetAction != null){
                if(!targetAction.isActionFinished()){
                    if(targetAction.isBusy()) {
                        //the segment has been followed for at least one loop time, but the action isn't finished yet.
                        if (target.skipActionWhenSegmentFinished) {
                            if(target.stopActionWhenSkipping) {
                                targetAction.stopAction();
                            }
                        } else {
                            jumpToNextSegment = false;
                        }
                    }else{
                        //the segment is being jumped, we can force it to not jump and go to the segment.
                        if (target.skipActionWhenSegmentFinished) {
                            //we do not have to execute the action because it is going to be interrupted in the end anyway.
                        } else {
                            jumpToNextSegment = false;
                        }
                    }
                }
            }
            if (jumpToNextSegment) {
                this.m_PathCursor++;
            }
        } while (jumpToNextSegment && this.m_PathCursor < this.m_Path.size() - 1);
        if (this.m_PathCursor >= this.m_Path.size() - 1) {return null;}

        PurePursuitWayPoint target = this.m_Path.get(this.m_PathCursor + 1);

        //Calculate Follow Speed
        double currentSegmentEndSpeed = target.getEndFollowNormalizedSpeed();
        if(currentSegmentEndSpeed == 0){
            currentSegmentEndSpeed = this.getFollowSpeed();
        }
        double currentSegmentStartSpeed = 0;
        double distSegmentStartEnd = 0;
        if(this.m_PathCursor < 0){
            currentSegmentStartSpeed = this.getStartSpeed();
            distSegmentStartEnd = target.distanceTo(XYPlaneCalculations.ORIGIN_POINT);
        }else{
            PurePursuitWayPoint targetStart = this.m_Path.get(this.m_PathCursor);
            distSegmentStartEnd = target.distanceTo(targetStart);
            currentSegmentStartSpeed = targetStart.getEndFollowNormalizedSpeed() == 0 ? this.getFollowSpeed() : targetStart.getEndFollowNormalizedSpeed();
        }

        double followSpeed;
        if(currentSegmentEndSpeed != currentSegmentStartSpeed) {
            double distToEndPoint = distToTarget;
            double progress = (distSegmentStartEnd - distToEndPoint) / distSegmentStartEnd;
            if (progress >= 0) {
                progress = Math.sqrt(progress);
            } else {
                progress = -Math.sqrt(-progress);
            }
            followSpeed = currentSegmentStartSpeed * progress + currentSegmentEndSpeed * (1.0 - progress);
        }else{
            followSpeed = currentSegmentStartSpeed;
        }
        //End Calculating Follow Speed

        if (target instanceof PurePursuitEndPoint && distToTarget < target.getFollowDistance()) {
            return new FollowInformation(target,target,followSpeed);
        } else if (target instanceof PurePursuitHeadingInterpolationWayPoint) {
            if(distToTarget < target.getFollowDistance()) {
                return new FollowInformation(target, target, followSpeed);
            }else{
                return new FollowInformation(target,getCurrentSegmentFollowPoint(currentOffset,target),followSpeed);
            }
        } else {
            return new FollowInformation(target,getCurrentSegmentFollowPoint(currentOffset,target),followSpeed);
        }
    }

    protected RobotPoint2D getCurrentSegmentFollowPoint(RobotPose2D currentOffset, PurePursuitWayPoint wayPoint){
        RobotPoint2D firstPoint;
        RobotPoint2D secondPoint;
        if(this.m_PathCursor == -1){
            firstPoint = new RobotPoint2D(0,0);
        }else{
            firstPoint = this.m_Path.get(this.m_PathCursor);
        }
        secondPoint = this.m_Path.get(this.m_PathCursor);
        RobotPoint2D centerPoint = XYPlaneCalculations.nearestPointOnLine(currentOffset.toPoint2D(),firstPoint,secondPoint);
        ArrayList<RobotPoint2D> intersectionPoints = XYPlaneCalculations.lineCircleIntersections(centerPoint,wayPoint.getFollowDistance(),firstPoint,secondPoint);
        RobotPoint2D closestPoint = XYPlaneCalculations.getNearestPoint(intersectionPoints,secondPoint);
        return closestPoint;
    }

    @Override
    protected RobotPose2D __getSupposedTaskFinishPos() {
        return null;
    }

    @Override
    public void drawPath(Canvas dashboardCanvas) {

    }
}
