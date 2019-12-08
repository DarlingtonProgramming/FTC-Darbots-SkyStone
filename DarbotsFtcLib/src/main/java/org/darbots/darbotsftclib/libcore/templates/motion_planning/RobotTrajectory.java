package org.darbots.darbotsftclib.libcore.templates.motion_planning;

import org.darbots.darbotsftclib.libcore.motion_planning.profiles.MotionProfile;
import org.darbots.darbotsftclib.libcore.motion_planning.trajectories.TrajectoryMotionState;

public interface RobotTrajectory {
    RobotPath getPath();
    MotionProfile getMotionProfile();
    RobotMotionProfilingIterator<TrajectoryMotionState,?> getIterator();
    TrajectoryMotionState getEndState();
}
