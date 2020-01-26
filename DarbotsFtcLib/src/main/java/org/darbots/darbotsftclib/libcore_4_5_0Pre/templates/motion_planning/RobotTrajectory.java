package org.darbots.darbotsftclib.libcore_4_5_0Pre.templates.motion_planning;

import org.darbots.darbotsftclib.libcore_4_5_0Pre.motion_planning.trajectories.TrajectoryMotionState;

public interface RobotTrajectory {
    RobotMotionProfilingIterator<TrajectoryMotionState,?> getIterator();
    TrajectoryMotionState getEndState();
}
