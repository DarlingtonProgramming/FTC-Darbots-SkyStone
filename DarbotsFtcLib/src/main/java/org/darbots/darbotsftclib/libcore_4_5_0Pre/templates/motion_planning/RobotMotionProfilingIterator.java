package org.darbots.darbotsftclib.libcore_4_5_0Pre.templates.motion_planning;

import java.util.NoSuchElementException;

public interface RobotMotionProfilingIterator<StatusType, SegmentType> {
    StatusType forward(double deltaDuration) throws NoSuchElementException;
    StatusType backward(double deltaBackwardDuration) throws NoSuchElementException;
    StatusType current();
    SegmentType currentSegment();
    double getCurrentDuration();
    double getTotalDuration();
}
