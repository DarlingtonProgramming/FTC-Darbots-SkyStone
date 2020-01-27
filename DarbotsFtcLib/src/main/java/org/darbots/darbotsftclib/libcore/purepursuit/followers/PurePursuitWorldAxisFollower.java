package org.darbots.darbotsftclib.libcore.purepursuit.followers;

import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPose2D;
import org.darbots.darbotsftclib.libcore.purepursuit.utils.PurePursuitWayPoint;

import java.util.ArrayList;

public class PurePursuitWorldAxisFollower extends PurePursuitPathFollower {
    public PurePursuitWorldAxisFollower(ArrayList<PurePursuitWayPoint> path, double normalizedStartSpeed, double normalizedFollowSpeed, double normalizedAngleSpeed, double preferredAngle) {
        super(path, normalizedStartSpeed, normalizedFollowSpeed, normalizedAngleSpeed, preferredAngle);
    }

    public PurePursuitWorldAxisFollower(PurePursuitWorldAxisFollower oldFollower) {
        super(oldFollower);
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
}
