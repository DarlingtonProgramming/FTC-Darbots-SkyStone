package org.darbots.darbotsftclib.libcore.motion_planning.trajectories;

import org.darbots.darbotsftclib.libcore.motion_planning.profiles.MotionProfile;
import org.darbots.darbotsftclib.libcore.templates.motion_planning.RobotMotionProfilingIterator;
import org.darbots.darbotsftclib.libcore.templates.motion_planning.RobotPath;
import org.darbots.darbotsftclib.libcore.templates.motion_planning.RobotTrajectory;

import java.util.ArrayList;

public class SimpleTrajectory implements RobotTrajectory {
    private RobotPath m_Path;
    private MotionProfile m_MotionProfile;
    private ArrayList<TrajectoryMotionSegment> m_MotionSegments;
    private double m_Resolution = 0;

    public SimpleTrajectory(RobotPath path, MotionProfile profile, double resolution){
        this.m_Path = path;
        this.m_MotionProfile = profile;
        this.m_MotionSegments = new ArrayList<TrajectoryMotionSegment>();
        this.m_Resolution = resolution;
    }

    public SimpleTrajectory(SimpleTrajectory oldTrajectory){
        this.m_Path = oldTrajectory.m_Path;
        this.m_MotionProfile = oldTrajectory.m_MotionProfile;
        this.m_MotionSegments = new ArrayList<TrajectoryMotionSegment>(oldTrajectory.m_MotionSegments);
        this.m_Resolution = oldTrajectory.m_Resolution;
    }

    @Override
    public RobotPath getPath() {
        return m_Path;
    }

    @Override
    public MotionProfile getMotionProfile() {
        return m_MotionProfile;
    }

    @Override
    public RobotMotionProfilingIterator<TrajectoryMotionState, TrajectoryMotionSegment> getIterator() {
        return new SimpleTrajectoryIterator(this);
    }

    @Override
    public TrajectoryMotionState getEndState() {
        if(this.m_MotionSegments.isEmpty()){
            return new TrajectoryMotionState(0,0,0,0,0);
        }else{
            TrajectoryMotionSegment lastSegment = this.m_MotionSegments.get(this.m_MotionSegments.size() - 1);
            return lastSegment.getStatusAt(lastSegment.duration);
        }
    }

    public ArrayList<TrajectoryMotionSegment> getMotionSegments(){
        return this.m_MotionSegments;
    }

    public double getResolution(){
        return this.m_Resolution;
    }
}
