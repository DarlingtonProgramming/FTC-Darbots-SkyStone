package org.darbots.darbotsftclib.libcore.purepursuit.followers;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.robotcore.util.Range;

import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPoint2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPose2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.XYPlaneCalculations;
import org.darbots.darbotsftclib.libcore.purepursuit.utils.PurePursuitHeadingInterpolationWayPoint;
import org.darbots.darbotsftclib.libcore.purepursuit.utils.PurePursuitWayPoint;
import org.darbots.darbotsftclib.libcore.templates.chassis_related.RobotMotionSystemTask;

import java.util.ArrayList;

public class PurePursuitPathWorldAxisFollower extends RobotMotionSystemTask {
    public static class FollowInformation{
        public PurePursuitWayPoint purePursuitWayPoint;
        public RobotPoint2D pursuitPoint;

        public FollowInformation(PurePursuitWayPoint wayPoint, RobotPoint2D followPoint){
            this.pursuitPoint = followPoint;
            this.purePursuitWayPoint = wayPoint;
        }
        public FollowInformation(FollowInformation oldInfo){
            this.pursuitPoint = oldInfo.pursuitPoint;
            this.purePursuitWayPoint = oldInfo.purePursuitWayPoint;
        }
    }

    private ArrayList<PurePursuitWayPoint> m_Path;
    private int m_PathCursor;
    private double m_EndingPositionErrorRange = 3.0;
    private double m_FollowSpeedNormalized;
    private double m_AngleSpeedNormalized;
    private double m_PreferredAngle;

    public PurePursuitPathWorldAxisFollower(ArrayList<PurePursuitWayPoint> path, double normalizedFollowSpeed, double normalizedAngleSpeed, double preferredAngle){
        this.m_Path = new ArrayList<>();
        this.m_Path.addAll(path);
        this.setFollowSpeed(normalizedFollowSpeed);
        this.setAngleSpeed(normalizedAngleSpeed);
        this.setPreferredAngle(preferredAngle);
    }

    public PurePursuitPathWorldAxisFollower(PurePursuitPathWorldAxisFollower oldFollower){
        super(oldFollower);
        this.m_Path = new ArrayList<>();
        this.m_Path.addAll(oldFollower.m_Path);
        this.m_FollowSpeedNormalized = oldFollower.m_FollowSpeedNormalized;
        this.m_AngleSpeedNormalized = oldFollower.m_AngleSpeedNormalized;
        this.m_PreferredAngle = oldFollower.m_PreferredAngle;
        this.m_EndingPositionErrorRange = oldFollower.m_EndingPositionErrorRange;
    }

    public double getEndingPositionErrorRange(){
        return this.m_EndingPositionErrorRange;
    }

    public void setEndingPositionErrorRange(double endingPositionErrorRange){
        this.m_EndingPositionErrorRange = Math.abs(endingPositionErrorRange);
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
        RobotPose2D currentOffset = this.getMotionSystem().getCurrentPosition();
        FollowInformation followInfo = this.getFollowInformation(currentOffset);
        if(followInfo == null){
            this.stopTask();
            return;
        }
        this.gotoPoint(currentOffset,followInfo.purePursuitWayPoint,followInfo.pursuitPoint,this.m_FollowSpeedNormalized,this.m_AngleSpeedNormalized,this.m_PreferredAngle);
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
        boolean jumpToNextSegment;
        do {
            boolean lastSegment = this.m_PathCursor + 1 == this.m_Path.size() - 1;
            jumpToNextSegment = false;
            PurePursuitWayPoint target = this.m_Path.get(this.m_PathCursor + 1);

            if (lastSegment) {
                if (currentOffset.toPoint2D().distanceTo(target) <= this.m_EndingPositionErrorRange) {
                    jumpToNextSegment = true;
                }
                if(target instanceof PurePursuitHeadingInterpolationWayPoint) {
                    PurePursuitHeadingInterpolationWayPoint ptTarget = (PurePursuitHeadingInterpolationWayPoint) target;
                    if (!(Math.abs(XYPlaneCalculations.normalizeDeg(currentOffset.getRotationZ() - ptTarget.getDesiredHeading())) <= ptTarget.getAllowedHeadingError())) {
                        jumpToNextSegment = false;
                    }
                }
            }else if (target instanceof PurePursuitHeadingInterpolationWayPoint) {
                PurePursuitHeadingInterpolationWayPoint ptTarget = (PurePursuitHeadingInterpolationWayPoint) target;
                if ((Math.abs(XYPlaneCalculations.normalizeDeg(currentOffset.getRotationZ() - ptTarget.getDesiredHeading())) <= ptTarget.getAllowedHeadingError())) {
                    jumpToNextSegment = true;
                }
            }else {
                if (currentOffset.toPoint2D().distanceTo(target) <= target.getFollowDistance()) {
                    jumpToNextSegment = true;
                }
            }


            if (jumpToNextSegment) {
                this.m_PathCursor++;
            }
        } while (jumpToNextSegment && this.m_PathCursor < this.m_Path.size() - 1);
        if (this.m_PathCursor >= this.m_Path.size() - 1) {return null;}

        PurePursuitWayPoint target = this.m_Path.get(this.m_PathCursor + 1);
        boolean lastSegment = this.m_PathCursor + 1 == this.m_Path.size() - 1;

        if (lastSegment && currentOffset.toPoint2D().distanceTo(target) < target.getFollowDistance()) {
            return new FollowInformation(target,target);
        } else if (target instanceof PurePursuitHeadingInterpolationWayPoint) {
            return new FollowInformation(target,target);
        } else {
            return new FollowInformation(target,getCurrentSegmentFollowPoint(currentOffset,target));
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
