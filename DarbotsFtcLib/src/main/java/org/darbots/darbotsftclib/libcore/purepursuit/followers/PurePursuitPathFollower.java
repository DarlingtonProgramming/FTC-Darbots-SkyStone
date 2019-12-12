package org.darbots.darbotsftclib.libcore.purepursuit.followers;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPoint2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPose2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.XYPlaneCalculations;
import org.darbots.darbotsftclib.libcore.motion_planning.profiles.MotionProfile;
import org.darbots.darbotsftclib.libcore.motion_planning.profiles.MotionProfileGenerator;
import org.darbots.darbotsftclib.libcore.motion_planning.profiles.MotionState;
import org.darbots.darbotsftclib.libcore.purepursuit.utils.PurePursuitWayPoint;
import org.darbots.darbotsftclib.libcore.templates.chassis_related.RobotMotionSystemTask;

import java.util.ArrayList;

public class PurePursuitPathFollower extends RobotMotionSystemTask {
    private ArrayList<PurePursuitWayPoint> m_PathToFollow;
    private double m_MaximumAcceleration;
    private double m_StartSpeed, m_CruiseSpeed, m_EndSpeed, m_AngleSpeed;
    private MotionProfile m_ProfileToReachEndSpeed;
    private double m_ProfileToReachEndSpeedTotalDistance;
    private double m_ProfileToReachEndSpeedTotalDuration;
    private double m_LastSpeed;
    private ElapsedTime m_TimeBetweenCalls;
    private ElapsedTime m_TimeForEnding;
    private boolean m_EndingStarted;
    private double m_FollowRadius;
    private int m_LastFollowedSegment = 0;
    private double m_ErrorRange = 1;
    private double m_ErrorDuration = 2;
    private double m_PreferredAngle = 0;

    public PurePursuitPathFollower(ArrayList<PurePursuitWayPoint> wayPoints, double maximumAcceleration, double startSpeed, double cruiseSpeed, double endSpeed, double angleSpeed, double followRadius, double preferredAngle){
        this.m_PathToFollow = new ArrayList<>(wayPoints);
        this.m_MaximumAcceleration = Math.abs(maximumAcceleration);
        this.m_StartSpeed = Math.abs(startSpeed);
        this.m_CruiseSpeed = Math.abs(cruiseSpeed);
        this.m_EndSpeed = Math.abs(endSpeed);
        this.m_AngleSpeed = Math.abs(angleSpeed);
        this.m_FollowRadius = Math.abs(followRadius);
        this.m_PreferredAngle = XYPlaneCalculations.normalizeDeg(preferredAngle);
        this.__recalculateProfileToReachEndSpeed();
        this.updateWayPoints();
    }

    public PurePursuitPathFollower(PurePursuitPathFollower follower){
        this.m_PathToFollow = new ArrayList<>(follower.m_PathToFollow);
        this.m_MaximumAcceleration = follower.m_MaximumAcceleration;
        this.m_StartSpeed = follower.m_StartSpeed;
        this.m_CruiseSpeed = follower.m_CruiseSpeed;
        this.m_EndSpeed = follower.m_EndSpeed;
        this.m_AngleSpeed = follower.m_AngleSpeed;
        this.m_ProfileToReachEndSpeed = new MotionProfile(follower.m_ProfileToReachEndSpeed);
        this.m_ProfileToReachEndSpeedTotalDistance = follower.m_ProfileToReachEndSpeedTotalDistance;
        this.m_FollowRadius = follower.m_FollowRadius;
        this.m_ErrorRange = follower.m_ErrorRange;
        this.m_ErrorDuration = follower.m_ErrorDuration;
        this.m_PreferredAngle = follower.m_PreferredAngle;
    }

    public ArrayList<PurePursuitWayPoint> getWayPoints(){
        return this.m_PathToFollow;
    }

    public void updateWayPoints(){
        if(this.m_PathToFollow.isEmpty()){
            this.m_PathToFollow.add(new PurePursuitWayPoint(0,0));
        }else{
            PurePursuitWayPoint firstPoint = this.m_PathToFollow.get(0);
            if(firstPoint.X != 0 || firstPoint.Y != 0){
                this.m_PathToFollow.add(0,new PurePursuitWayPoint(0,0));
            }
        }
    }

    public boolean isPathValid(){
        if(this.m_PathToFollow.isEmpty()){
            return false;
        }else if(this.m_PathToFollow.size() == 1){
            PurePursuitWayPoint onlyPoint = this.m_PathToFollow.get(0);
            if(onlyPoint.X == 0 && onlyPoint.Y == 0){
                return false;
            }
        }
        return true;
    }

    public double getPreferredAngle(){
        return this.m_PreferredAngle;
    }

    public void setPreferredAngle(double PreferredAngle){
        this.m_PreferredAngle = XYPlaneCalculations.normalizeDeg(PreferredAngle);
    }

    public double getAngularSpeed(){
        return this.m_AngleSpeed;
    }

    public void setAngularSpeed(double AngularSpeed){
        this.m_AngleSpeed = Math.abs(AngularSpeed);
    }

    public double getErrorDuration(){
        return this.m_ErrorDuration;
    }

    public void setErrorDuration(double ErrorDuration){
        this.m_ErrorDuration = Math.abs(ErrorDuration);
    }

    public double getErrorRange(){
        return this.m_ErrorRange;
    }

    public void setErrorRange(double ErrorRange){
        this.m_ErrorRange = Math.abs(ErrorRange);
    }

    public double getStartSpeed(){
        return m_StartSpeed;
    }

    public void setStartSpeed(double startSpeed){
        this.m_StartSpeed = Math.abs(startSpeed);
    }

    public double getCruiseSpeed(){
        return this.m_CruiseSpeed;
    }

    public void setCruiseSpeed(double CruiseSpeed){
        this.m_CruiseSpeed = Math.abs(CruiseSpeed);
        this.__recalculateProfileToReachEndSpeed();
    }

    public double getMaximumAcceleration(){
        return this.m_MaximumAcceleration;
    }

    public void setMaximumAcceleration(double MaximumAccel){
        this.m_MaximumAcceleration = Math.abs(MaximumAccel);
        this.__recalculateProfileToReachEndSpeed();
    }

    public double getEndSpeed(){
        return this.m_EndSpeed;
    }

    public void setEndSpeed(double EndSpeed){
        this.m_EndSpeed = Math.abs(EndSpeed);
        this.__recalculateProfileToReachEndSpeed();
    }

    public double getFollowRadius(){
        return this.m_FollowRadius;
    }

    public void setFollowRadius(double FollowRadius){
        this.m_FollowRadius = Math.abs(FollowRadius);
    }

    private void __recalculateProfileToReachEndSpeed(){
        this.m_EndingStarted = false;
        this.m_ProfileToReachEndSpeed = MotionProfileGenerator.generateMotionProfileFromOneSpeedToAnother(this.m_MaximumAcceleration,0,0,0,m_CruiseSpeed,m_EndSpeed);
        this.m_ProfileToReachEndSpeedTotalDistance = this.m_ProfileToReachEndSpeed.getTotalDistance();
        this.m_ProfileToReachEndSpeedTotalDuration = this.m_ProfileToReachEndSpeed.getTotalDuration();
    }

    @Override
    protected void __startTask() {
        this.m_LastSpeed = this.m_StartSpeed;
        this.m_TimeBetweenCalls = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        this.m_LastFollowedSegment = 0;
        this.m_TimeForEnding = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        this.m_EndingStarted = false;
    }

    @Override
    protected void __taskFinished() {
        this.m_TimeBetweenCalls = null;
        this.m_TimeForEnding = null;
    }

    @Override
    protected void __updateStatus() {
        double timeBetweenCall = m_TimeBetweenCalls.seconds();
        m_TimeBetweenCalls.reset();
        if(!isPathValid()){
            this.stopTask();
            return;
        }

        PurePursuitWayPoint endPoint = this.m_PathToFollow.get(this.m_PathToFollow.size() - 1);
        RobotPose2D currentOffset = this.getRelativePositionOffsetSinceStart();
        PurePursuitWayPoint pursuitPoint = null;
        double pursuitSpeed = 0;

        double distanceToEndPoint = currentOffset.toPoint2D().distanceTo(endPoint);

        double timeForEnding = 0;

        if(this.m_EndingStarted){
            timeForEnding = this.m_TimeForEnding.seconds();
        }else if((distanceToEndPoint <= this.m_FollowRadius && this.m_ProfileToReachEndSpeedTotalDistance == 0) || distanceToEndPoint <= this.m_ProfileToReachEndSpeedTotalDistance){
            this.m_EndingStarted = true;
            this.m_TimeForEnding.reset();
        }

        if(this.m_EndingStarted){
            RobotPoint2D error = XYPlaneCalculations.getRelativePosition(currentOffset,pursuitPoint);
            double timeToEnd;
            if(this.m_ProfileToReachEndSpeedTotalDistance == 0){
                timeToEnd = this.m_FollowRadius / ((this.m_CruiseSpeed + this.m_EndSpeed) / 2.0);
            }else{
                timeToEnd = this.m_ProfileToReachEndSpeedTotalDuration;
            }
            if(timeForEnding >= timeToEnd + this.getErrorDuration()){
                this.stopTask();
                return;
            }
            if(Math.abs(error.X) <= this.m_ErrorRange && Math.abs(error.Y) <= this.m_ErrorRange){
                this.stopTask();
                return;
            }
        }

        if(this.m_EndingStarted && distanceToEndPoint <= this.m_ProfileToReachEndSpeedTotalDistance){
            this.m_LastFollowedSegment = this.m_PathToFollow.size() - 2;
            pursuitPoint = endPoint;
            timeForEnding = this.m_TimeForEnding.seconds();
            MotionState currentMotionState = this.m_ProfileToReachEndSpeed.getMotionStateAt(timeForEnding);
            pursuitSpeed = currentMotionState.velocity;
        }else{
            if(this.m_LastSpeed != this.m_CruiseSpeed){
                if(this.m_CruiseSpeed > this.m_LastSpeed){
                    pursuitSpeed = this.m_LastSpeed + this.m_MaximumAcceleration * timeBetweenCall;
                    pursuitSpeed = Range.clip(pursuitSpeed,this.m_LastSpeed,this.m_CruiseSpeed);
                }else{
                    pursuitSpeed = this.m_LastSpeed - this.m_MaximumAcceleration * timeBetweenCall;
                    pursuitSpeed = Range.clip(pursuitSpeed,this.m_CruiseSpeed,this.m_LastSpeed);
                }
            }else{
                pursuitSpeed = this.m_CruiseSpeed;
            }
            pursuitPoint = getFollowPoint(currentOffset,this.m_PreferredAngle);
        }
        if(distanceToEndPoint <= this.m_FollowRadius){
            pursuitPoint = endPoint;
        }
        double normalizedPursuitSpeed = pursuitSpeed / this.getMotionSystem().calculateMaxLinearSpeedInCMPerSec();
        double normalizedAngularSpeed = this.m_AngleSpeed / this.getMotionSystem().calculateMaxAngularSpeedInDegPerSec();
        this.gotoPosition(currentOffset,pursuitPoint,normalizedPursuitSpeed,normalizedAngularSpeed,m_PreferredAngle);
        this.m_LastSpeed = pursuitSpeed;
    }

    @Override
    protected RobotPose2D __getSupposedTaskFinishPos() {
        PurePursuitWayPoint endPoint = this.m_PathToFollow.get(this.m_PathToFollow.size() - 1);
        RobotPose2D supposedEnd = new RobotPose2D(endPoint.X,endPoint.Y,Double.NaN);
        return supposedEnd;
    }

    protected PurePursuitWayPoint getFollowPoint(RobotPose2D currentOffset, double preferredAngle) {
        if (!isPathValid()) {
            return new PurePursuitWayPoint(0, 0);
        }
        {
            PurePursuitWayPoint followPoint = new PurePursuitWayPoint(0, 0);
            boolean foundPoint = false;
            double closestAngle = 10000000;
            for (int i = m_LastFollowedSegment; i < this.m_PathToFollow.size() - 1; i++) {
                PurePursuitWayPoint startPoint = this.m_PathToFollow.get(i);
                PurePursuitWayPoint endPoint = this.m_PathToFollow.get(i + 1);
                ArrayList<RobotPoint2D> intersections = XYPlaneCalculations.lineSegmentCircleIntersections(currentOffset.toPoint2D(), this.m_FollowRadius, startPoint, endPoint);
                for (RobotPoint2D thisIntersection : intersections) {
                    RobotPoint2D currentRobotAxisTarget = XYPlaneCalculations.getRelativePosition(currentOffset, thisIntersection);
                    double targetPointAngle = Math.toDegrees(Math.atan2(currentRobotAxisTarget.Y, currentRobotAxisTarget.X));
                    double wantedAngleInCurrentRobotAxis = targetPointAngle + preferredAngle;
                    if (Math.abs(closestAngle) > Math.abs(wantedAngleInCurrentRobotAxis)) {
                        closestAngle = wantedAngleInCurrentRobotAxis;
                        followPoint.setValues(thisIntersection);
                        this.m_LastFollowedSegment = i;
                        foundPoint = true;
                    }
                }
            }
            if (foundPoint) {
                return followPoint;
            }
        }
        {
            ArrayList<RobotPoint2D> nearestPoints = new ArrayList<>();
            for (int i = m_LastFollowedSegment; i < this.m_PathToFollow.size() - 1; i++) {
                PurePursuitWayPoint startPoint = this.m_PathToFollow.get(i);
                PurePursuitWayPoint endPoint = this.m_PathToFollow.get(i + 1);
                RobotPoint2D nearestPoint = XYPlaneCalculations.nearestPointOnLineSegment(currentOffset.toPoint2D(), startPoint, endPoint);
                if (nearestPoint != null) {
                    nearestPoints.add(nearestPoint);
                }
            }
            if (nearestPoints.isEmpty()) {
                return new PurePursuitWayPoint(0, 0);
            }
            double closestDistance = 1000000;
            PurePursuitWayPoint pursuitPoint = new PurePursuitWayPoint(0, 0);
            for (RobotPoint2D thisPoint : nearestPoints) {
                double currentDistance = thisPoint.distanceTo(currentOffset.toPoint2D());
                if (currentDistance < closestDistance) {
                    pursuitPoint.setValues(thisPoint);
                    closestDistance = currentDistance;
                }
            }
            return new PurePursuitWayPoint(pursuitPoint);
        }
    }

    protected void gotoPosition(RobotPose2D currentOffset, RobotPoint2D targetPoint, double normalizedLinearSpeed, double normalizedAngularSpeed, double preferredAngle){
        RobotPoint2D currentRobotAxisTarget = XYPlaneCalculations.getRelativePosition(currentOffset,targetPoint);
        double targetPointAngle = Math.toDegrees(Math.atan2(currentRobotAxisTarget.Y,currentRobotAxisTarget.X));
        double wantedAngleInCurrentRobotAxis = targetPointAngle + preferredAngle;
        RobotPose2D supposedPose = new RobotPose2D(currentRobotAxisTarget,wantedAngleInCurrentRobotAxis);
        this.updateSupposedPos(supposedPose);

        double XAndYSpeed = Math.abs(currentRobotAxisTarget.X) + Math.abs(currentRobotAxisTarget.Y);
        double xSpeed = currentRobotAxisTarget.X / XAndYSpeed, ySpeed = currentRobotAxisTarget.Y / XAndYSpeed;
        xSpeed *= normalizedLinearSpeed;
        ySpeed *= normalizedLinearSpeed;
        double angularSpeed = Range.clip(wantedAngleInCurrentRobotAxis / 30.0,-1.0,1.0) * normalizedAngularSpeed;

        double distanceToTarget = currentRobotAxisTarget.distanceTo(new RobotPoint2D(0,0));
        if(distanceToTarget < 10){
            angularSpeed = 0;
        }

        this.getMotionSystem().setRobotSpeed(
                xSpeed * this.getMotionSystem().calculateMaxLinearXSpeedInCMPerSec(),
                ySpeed * this.getMotionSystem().calculateMaxLinearYSpeedInCMPerSec(),
                angularSpeed * this.getMotionSystem().calculateMaxAngularSpeedInDegPerSec());
    }
}
