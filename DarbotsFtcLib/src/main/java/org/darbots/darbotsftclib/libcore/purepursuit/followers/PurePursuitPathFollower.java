package org.darbots.darbotsftclib.libcore.purepursuit.followers;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPoint2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPose2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.XYPlaneCalculations;
import org.darbots.darbotsftclib.libcore.purepursuit.waypoints.PurePursuitEndPoint;
import org.darbots.darbotsftclib.libcore.purepursuit.waypoints.PurePursuitHeadingInterpolationWayPoint;
import org.darbots.darbotsftclib.libcore.purepursuit.waypoints.PurePursuitWayPoint;
import org.darbots.darbotsftclib.libcore.templates.DarbotsAction;
import org.darbots.darbotsftclib.libcore.templates.chassis_related.RobotMotionSystemTask;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

public class PurePursuitPathFollower {
    public static class FollowInformation{
        public PurePursuitWayPoint purePursuitWayPoint;
        public RobotPoint2D pursuitPoint;
        public double normalizedFollowSpeed;
        public boolean pidEnabled;

        public FollowInformation(PurePursuitWayPoint wayPoint, RobotPoint2D followPoint, double normalizedFollowSpeed, boolean PIDEnabled){
            this.pursuitPoint = followPoint;
            this.purePursuitWayPoint = wayPoint;
            this.normalizedFollowSpeed = Math.abs(normalizedFollowSpeed);
            this.pidEnabled = PIDEnabled;
        }
        public FollowInformation(FollowInformation oldInfo){
            this.pursuitPoint = oldInfo.pursuitPoint;
            this.purePursuitWayPoint = oldInfo.purePursuitWayPoint;
            this.normalizedFollowSpeed = oldInfo.normalizedFollowSpeed;
            this.pidEnabled = oldInfo.pidEnabled;
        }
    }


}
